/*
 - how to compile:
        gcc disk.c -o disk -lm 
 
 - how to run:
        ./disk [options]
        
 - Available options:
  -s  SEED                : Set random seed (default 0)
  -A  ADDR_LIST           : Comma-separated block list, or -1 for random (e.g. 5,10,7)
  -D  N,MAX,MIN           : Auto-generate N requests (if -A is -1)
  -S  SEEK_SPEED          : Disk seek speed (default 1)
  -R  ROTATE_SPEED        : Disk rotation speed (default 1)
  -p  POLICY              : Scheduling policy [FIFO | SSTF | SATF | BSATF]
  -w  WINDOW_SIZE         : Scheduling window (-1 = all requests)
  -z  Z1,Z2,Z3            : Disk zoning configuration (default 30,30,30)
  -L  LATE_ADDR_LIST      : Late request list (comma-separated) or -1
  -M  N,MAX,MIN           : Generate late requests randomly
  -C                      : Enable computation mode (prints seek/rotate/transfer times)

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <ctype.h>
#include <limits.h>


#define MAX_TRACKS 3
#define MAX_BLOCKS 4096

int opt_seed = 0;
char opt_addr[256] = "-1";
char opt_addrDesc[64] = "5,-1,0";
double opt_seekSpeed = 1.0;
double opt_rotSpeed = 1.0;
int opt_skew = 0;
int opt_window = -1;
char opt_policy[32] = "FIFO";
int opt_compute = 0;
int opt_graphics = 0;
char opt_zoning[64] = "30,30,30";
char opt_lateAddr[256] = "-1";
char opt_lateDesc[64] = "0,-1,0";

/* ---------- Disk layout derived from zoning ---------- */
int zoning[MAX_TRACKS];
int blockAngleOffset[MAX_TRACKS];
int blockToTrack[MAX_BLOCKS];
int blockToAngle[MAX_BLOCKS];
int maxBlock = 0;

/* ---------- Request queue & states ---------- */
typedef enum { STATE_NULL=0, STATE_SEEK=1, STATE_ROTATE=2, STATE_XFER=3, STATE_DONE=4 } rstate_t;
typedef struct { int block; rstate_t state; } request_t;
request_t *requestQueue = NULL;
int requestQueueLen = 0;

/* Late requests */
int *lateRequests = NULL;
int lateCount = 0;
int lateTotal = 0;

/* --- Simulator state & stats --- */
int armTrack = 0;
double angle_deg = 0.0;         /* current disk angle in degrees */
double sim_time = 0.0;
double seekTotal = 0.0, rotTotal = 0.0, xferTotal = 0.0;
int processedCount = 0;

/* scheduling window / fairness */
int currWindow = -1;
int fairWindow = -1;

/* helper: trim spaces */
static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    if (*s == 0) return s;
    char *end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
    return s;
}

/* parse zoning string "30,30,30" into zoning[] */
void parse_zoning() {
    char buf[64];
    strncpy(buf, opt_zoning, sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    char *tok = strtok(buf, ",");
    int i=0;
    while (tok && i<3) {
        zoning[i++] = atoi(tok);
        tok = strtok(NULL, ",");
    }
    while (i<3) zoning[i++]=30;
}

void build_block_layout() {
    parse_zoning();
    for (int t=0;t<MAX_TRACKS;t++) blockAngleOffset[t] = zoning[t] / 2;
    maxBlock = 0;
    for (int track=0; track<MAX_TRACKS; track++) {
        int angleOffset = 2 * blockAngleOffset[track];
        if (angleOffset <= 0) angleOffset = 1;
        for (int a=0; a<360; a+=angleOffset) {
            blockToTrack[maxBlock] = track;
            blockToAngle[maxBlock] = (a + 180) % 360;
            maxBlock++;
            if (maxBlock >= MAX_BLOCKS) break;
        }
    }
}

/* generate requests from addr string "b1,b2,..." */
int *parse_addr_list(const char *s, int *out_count) {
    if (!s) { *out_count = 0; return NULL; }
    char tmp[512]; strncpy(tmp, s, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
    char *p = tmp;
    int capacity = 32;
    int *arr = malloc(capacity * sizeof(int));
    int n = 0;
    char *tok = strtok(p, ",");
    while (tok) {
        char *t = trim(tok);
        if (strlen(t)>0) {
            int v = atoi(t);
            if (n >= capacity) { capacity *= 2; arr = realloc(arr, capacity * sizeof(int)); }
            arr[n++] = v;
        }
        tok = strtok(NULL, ",");
    }
    *out_count = n;
    return arr;
}

/* generate random requests according to addrDesc "num,max,min" */
int *generate_addrdesc(const char *desc, int *out_count) {
    int num=0, max=-1, min=0;
    char buf[64]; strncpy(buf, desc, sizeof(buf)-1); buf[sizeof(buf)-1]=0;
    if (sscanf(buf, "%d,%d,%d", &num, &max, &min) < 1) { *out_count = 0; return NULL; }
    if (max == -1) max = maxBlock;
    if (num <= 0) { *out_count = 0; return NULL; }
    int *arr = malloc(num * sizeof(int));
    for (int i=0;i<num;i++) {
        arr[i] = min + (rand() % max);
    }
    *out_count = num;
    return arr;
}

/* compute seek time: dist in tracks * (trackWidth / seekSpeed) */
double estimate_seek_time(int fromTrack, int toTrack) {
    int dist = abs(fromTrack - toTrack);
    double t = (40.0 / opt_seekSpeed) * dist;
    return t;
}

/* compute actual seek: we model instant move in event-driven sim by consuming that time and
   updating armTrack (no step-by-step). */
double do_seek(int toTrack) {
    double t = estimate_seek_time(armTrack, toTrack);
    armTrack = toTrack;
    sim_time += t;
    return t;
}

/* estimate rotation time given arriving angle and block/track:
   angleAtArrival = angle + (seekEst * rotateSpeed)
   rotDist = ((blockAngle - angleOffset) - angleAtArrival) normalized to [0,360)
   rotEst = rotDist / rotateSpeed
*/
double estimate_rotate_time_given_seek(double angle_now, double seekEst, int block) {
    double angleAtArrival = angle_now + (seekEst * opt_rotSpeed);
    while (angleAtArrival > 360.0) angleAtArrival -= 360.0;
    int track = blockToTrack[block];
    double angleOffset = blockAngleOffset[track];
    double target = (blockToAngle[block] - angleOffset);
    while (target < 0.0) target += 360.0;
    double rotDist = target - angleAtArrival;
    while (rotDist > 360.0) rotDist -= 360.0;
    while (rotDist < 0.0) rotDist += 360.0;
    double rotEst = rotDist / opt_rotSpeed;
    return rotEst;
}

/* do rotation (consume time, update angle_deg) */
double do_rotate(int block) {
    int track = blockToTrack[block];
    double angleOffset = blockAngleOffset[track];
    double target = blockToAngle[block] - angleOffset;
    while (target < 0.0) target += 360.0;
    double diff = fmod((target - angle_deg + 360.0), 360.0);
    double t = diff / opt_rotSpeed;
    sim_time += t;
    angle_deg = fmod(angle_deg + diff, 360.0);
    return t;
}

/* estimate transfer time: (angleOffset * 2.0) / rotateSpeed */
double estimate_transfer_time(int block) {
    int track = blockToTrack[block];
    double t = (blockAngleOffset[track] * 2.0) / opt_rotSpeed;
    return t;
}

/* do transfer (consume time and advance angle accordingly) */
double do_transfer(int block) {
    double t = estimate_transfer_time(block);
    sim_time += t;
    angle_deg = fmod(angle_deg + (t * opt_rotSpeed), 360.0);
    return t;
}

/* SATF estimator: iterate over given candidate list and pick min totalEst
   candidates is array of (block,index) pairs encoded as separate arrays.
*/
void DoSATF_on_list(int *blocks, int *indices, int n, int *out_block, int *out_index) {
    double best = -1.0;
    int best_block = -1, best_index = -1;
    for (int i=0;i<n;i++) {
        int block = blocks[i];
        int idx = indices[i];
        if (requestQueue[idx].state == STATE_DONE) continue;
        int track = blockToTrack[block];
        double seekEst = estimate_seek_time(armTrack, track);
        double rotEst = estimate_rotate_time_given_seek(angle_deg, seekEst, block);
        double xferEst = estimate_transfer_time(block);
        double total = seekEst + rotEst + xferEst;
        if (best < 0 || total < best) {
            best = total;
            best_block = block;
            best_index = idx;
        }
    }
    if (best_block == -1) {
        fprintf(stderr, "DoSATF_on_list: no candidate found (this should not happen)\n");
        exit(1);
    }
    *out_block = best_block;
    *out_index = best_index;
}

/* DoSATF over a slice of requestQueue [0..endIndex-1] */
void DoSATF_window(int endIndex, int *out_block, int *out_index) {
    int capacity = endIndex>0?endIndex:1;
    int *blocks = malloc(capacity * sizeof(int));
    int *indices = malloc(capacity * sizeof(int));
    int n = 0;
    for (int i=0;i<endIndex && i<requestQueueLen;i++) {
        if (requestQueue[i].state == STATE_DONE) continue;
        blocks[n] = requestQueue[i].block;
        indices[n] = i;
        n++;
    }
    if (n == 0) {
        free(blocks); free(indices);
        *out_block = -1; *out_index = -1; return;
    }
    DoSATF_on_list(blocks, indices, n, out_block, out_index);
    free(blocks); free(indices);
}

/* DoSSTF: return all blocks on nearest track(s) within given window */
int *DoSSTF_list(int endIndex, int *out_count, int **out_indices) {
    int minDist = INT_MAX;
    int capacity = endIndex>0?endIndex:1;
    int *blocks = malloc(capacity * sizeof(int));
    int *indices = malloc(capacity * sizeof(int));
    int n = 0;
    for (int i=0;i<endIndex && i<requestQueueLen;i++) {
        if (requestQueue[i].state == STATE_DONE) continue;
        int track = blockToTrack[requestQueue[i].block];
        int d = abs(armTrack - track);
        if (d < minDist) {
            n = 0;
            blocks[n] = requestQueue[i].block;
            indices[n] = i;
            n++;
            minDist = d;
        } else if (d == minDist) {
            blocks[n] = requestQueue[i].block;
            indices[n] = i;
            n++;
        }
    }
    *out_count = n;
    *out_indices = indices; /* caller must free */
    return blocks; /* caller must free blocks */
}

/* GetWindow: */
int GetWindow() {
    if (currWindow <= -1) return requestQueueLen;
    if (fairWindow != -1) {
        if (processedCount > 0 && (processedCount % fairWindow == 0)) {
            currWindow = currWindow + fairWindow;
            if (currWindow > requestQueueLen) currWindow = requestQueueLen;
        }
        return currWindow;
    }
    return currWindow;
}

/* Add a request (block) at end of requestQueue */
void AddRequestInt(int block) {
    requestQueue = realloc(requestQueue, (requestQueueLen+1)*sizeof(request_t));
    requestQueue[requestQueueLen].block = block;
    requestQueue[requestQueueLen].state = STATE_NULL;
    requestQueueLen++;
}

/* Parse comma separated ints into an int array (caller must free) */
int *parse_int_list(const char *s, int *out_n) {
    if (!s) { *out_n=0; return NULL; }
    char tmp[1024]; strncpy(tmp, s, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
    char *tok = strtok(tmp, ",");
    int cap = 16; int n=0;
    int *arr = malloc(cap*sizeof(int));
    while (tok) {
        char *t = trim(tok);
        if (strlen(t)>0) {
            if (n>=cap) { cap*=2; arr=realloc(arr, cap*sizeof(int)); }
            arr[n++] = atoi(t);
        }
        tok = strtok(NULL, ",");
    }
    *out_n = n;
    return arr;
}

/* Initialize requests from opt_addr or opt_addrDesc */
void init_requests_from_options() {
    if (strcmp(opt_addr, "-1") != 0) {
        int n=0;
        int *arr = parse_int_list(opt_addr, &n);
        for (int i=0;i<n;i++) AddRequestInt(arr[i]);
        free(arr);
    } else {
        /* use addrDesc */
        int num=0, mx=-1, mn=0;
        sscanf(opt_addrDesc, "%d,%d,%d", &num, &mx, &mn);
        if (mx == -1) mx = maxBlock;
        if (num <= 0) num = 0;
        for (int i=0;i<num;i++) {
            int r = mn + (rand() % mx);
            AddRequestInt(r);
        }
    }
    /* late requests */
    if (strcmp(opt_lateAddr, "-1") != 0) {
        int n=0; int *arr = parse_int_list(opt_lateAddr, &n);
        lateRequests = malloc(n*sizeof(int));
        for (int i=0;i<n;i++) lateRequests[i]=arr[i];
        lateTotal = n;
        free(arr);
    } else {
        int num=0, mx=-1, mn=0;
        sscanf(opt_lateDesc, "%d,%d,%d", &num, &mx, &mn);
        if (mx == -1) mx = maxBlock;
        if (num <= 0) num = 0;
        if (num>0) lateRequests = malloc(num*sizeof(int));
        for (int i=0;i<num;i++) lateRequests[i] = mn + (rand()%mx);
        lateTotal = num;
    }
}

/* Print OPTIONS block */
void print_options() {
    printf("OPTIONS seed %d\n", opt_seed);
    printf("OPTIONS addr %s\n", opt_addr);
    printf("OPTIONS addrDesc %s\n", opt_addrDesc);
    printf("OPTIONS seekSpeed %.0f\n", opt_seekSpeed);
    printf("OPTIONS rotateSpeed %.0f\n", opt_rotSpeed);
    printf("OPTIONS skew %d\n", opt_skew);
    printf("OPTIONS window %d\n", opt_window);
    printf("OPTIONS policy %s\n", opt_policy);
    printf("OPTIONS compute %d\n", opt_compute);
    printf("OPTIONS zoning %s\n", opt_zoning);
    printf("OPTIONS lateAddr %s\n", opt_lateAddr);
    printf("OPTIONS lateAddrDesc %s\n\n", opt_lateDesc);
}

/* Service next I/O: choose according to policy, then compute times and mark done */
void ServiceAllRequests() {
    /* initialize scheduling window vars */
    currWindow = opt_window;
    if (strcmp(opt_policy, "BSATF")==0 && opt_window != -1) fairWindow = opt_window;
    else fairWindow = -1;

    processedCount = 0;
    while (processedCount < requestQueueLen) {
        /* check termination (in case new late requests were added) */
        if (processedCount == requestQueueLen) break;

        /* decide next (currentBlock, currentIndex) */
        int currentBlock = -1, currentIndex = -1;

        if (strcmp(opt_policy, "FIFO")==0) {
            /* FIFO: pick next not-done in arrival order */
            for (int i=0;i<requestQueueLen;i++) {
                if (requestQueue[i].state != STATE_DONE) { currentBlock = requestQueue[i].block; currentIndex = i; break; }
            }
        } else if (strcmp(opt_policy, "SATF")==0 || strcmp(opt_policy, "BSATF")==0) {
            int endIndex = GetWindow();
            if (endIndex > requestQueueLen) endIndex = requestQueueLen;
            DoSATF_window(endIndex, &currentBlock, &currentIndex);
        } else if (strcmp(opt_policy, "SSTF")==0) {
            int endIndex = GetWindow();
            if (endIndex > requestQueueLen) endIndex = requestQueueLen;
            int count = 0;
            int *indices = NULL;
            int *blocks = DoSSTF_list(endIndex, &count, &indices);
            if (count <= 0) {
                /* fallback */
                for (int i=0;i<requestQueueLen;i++) if (requestQueue[i].state != STATE_DONE) { currentBlock = requestQueue[i].block; currentIndex = i; break; }
                if (indices) free(indices);
                if (blocks) free(blocks);
            } else {
                /* Now run SATF on that subset (blocks + their indices) */
                int *btemp = malloc(count * sizeof(int));
                int *itemp = malloc(count * sizeof(int));
                for (int k=0;k<count;k++) { btemp[k]=blocks[k]; itemp[k]=indices[k]; }
                DoSATF_on_list(btemp, itemp, count, &currentBlock, &currentIndex);
                free(btemp); free(itemp); free(blocks); free(indices);
            }
        } else {
            /* unknown policy -> default FIFO */
            for (int i=0;i<requestQueueLen;i++) {
                if (requestQueue[i].state != STATE_DONE) { currentBlock = requestQueue[i].block; currentIndex = i; break; }
            }
        }

        if (currentIndex == -1) {
            /* nothing found - break */
            break;
        }

        /* PlanSeek performed 
        */
        if (lateCount < lateTotal) {
            AddRequestInt(lateRequests[lateCount]);
            lateCount++;
        }

        /* Now actually compute seek/rotate/transfer for chosen request */
        int targetTrack = blockToTrack[currentBlock];

        double seekEst = estimate_seek_time(armTrack, targetTrack);
        double rotEst  = estimate_rotate_time_given_seek(angle_deg, seekEst, currentBlock);
        double xferEst = estimate_transfer_time(currentBlock);

        /* Execute them (update sim_time and state) */
        double realSeek = do_seek(targetTrack);
        double realRot  = do_rotate(currentBlock);
        double realXfer = do_transfer(currentBlock);

        /* mark done */
        requestQueue[currentIndex].state = STATE_DONE;
        processedCount++;
        /* update counters */
        processedCount = processedCount; /* no-op, kept for clarity */
        processedCount = processedCount; /* harmless */

        /* accumulate stats and print */
        seekTotal += realSeek;
        rotTotal  += realRot;
        xferTotal += realXfer;
        processedCount = processedCount; /* keep stable */

        /* increment processedCount (number completed) - note: processedCount counted above as well */
        /* We want processedCount as number of completed requests; compute fresh */
        int doneCount = 0;
        for (int i=0;i<requestQueueLen;i++) if (requestQueue[i].state==STATE_DONE) doneCount++;
        processedCount = doneCount;

        if (opt_compute) {
            double total = realSeek + realRot + realXfer;
            printf("Block:%4d  Seek:%5.0f  Rotate:%5.0f  Transfer:%5.0f  Total:%6.0f\n",
                   currentBlock, realSeek, realRot, realXfer, total);
        }
    }

    /* final totals */
    if (opt_compute) {
        printf("\nTOTALS      Seek:%5.0f  Rotate:%5.0f  Transfer:%5.0f  Total:%7.0f\n",
               seekTotal, rotTotal, xferTotal, sim_time);
    }else {
    printf("\nComputation mode disabled. Run with -C to see seek/rotate/transfer calculations.\n");
}
}

int main(int argc, char **argv) {
    for (int i=1;i<argc;i++) {
        if (strcmp(argv[i], "-s")==0 && i+1<argc) { opt_seed = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "-A")==0 && i+1<argc) { strncpy(opt_addr, argv[++i], sizeof(opt_addr)-1); continue; }
        if (strcmp(argv[i], "-D")==0 && i+1<argc) { strncpy(opt_addrDesc, argv[++i], sizeof(opt_addrDesc)-1); continue; }
        if (strcmp(argv[i], "-S")==0 && i+1<argc) { opt_seekSpeed = atof(argv[++i]); continue; }
        if (strcmp(argv[i], "-R")==0 && i+1<argc) { opt_rotSpeed = atof(argv[++i]); continue; }
        if (strcmp(argv[i], "-w")==0 && i+1<argc) { opt_window = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "-p")==0 && i+1<argc) { strncpy(opt_policy, argv[++i], sizeof(opt_policy)-1); continue; }
        if (strcmp(argv[i], "-z")==0 && i+1<argc) { strncpy(opt_zoning, argv[++i], sizeof(opt_zoning)-1); continue; }
        if (strcmp(argv[i], "-L")==0 && i+1<argc) { strncpy(opt_lateAddr, argv[++i], sizeof(opt_lateAddr)-1); continue; }
        if (strcmp(argv[i], "-M")==0 && i+1<argc) { strncpy(opt_lateDesc, argv[++i], sizeof(opt_lateDesc)-1); continue; }
        if (strcmp(argv[i], "-C")==0) { opt_compute = 1; continue; }
        if (strcmp(argv[i], "-G")==0) { opt_graphics = 1; continue; }
    }

    /* seed RNG */
    srand(opt_seed);

    /* print OPTIONS block */
    print_options();

    /* build disk layout */
    build_block_layout();

    /* initialize requests */
    init_requests_from_options();

    /* if no requests, exit gracefully */
    if (requestQueueLen == 0) {
        printf("No requests to service.\n");
        return 0;
    }

    /* do the service loop (event-driven) */
    ServiceAllRequests();

    /* cleanup */
    if (requestQueue) free(requestQueue);
    if (lateRequests) free(lateRequests);

    return 0;
}
