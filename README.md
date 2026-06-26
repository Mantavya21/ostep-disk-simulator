# OSTEP Disk Simulator in C

This is a C implemention of the disk simulator from the OSTEP (Operating Systems: Three Easy Pieces) book. We built this for our unversity project to basically mirror the functionality of the original `disk.py` script. 

It simulates a hard drive and calculates the seek, rotate, and transfer times for different disk schduling policies.

### How to Compile
You need to link the math library cause of the calculations, so just use `gcc`:
```bash
gcc disk.c -o disk -lm
