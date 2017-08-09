import rospy
import inc
import initEnv

'''heap.c'''

def keyless(cell1, cell2):
    global keylength
    for keyindex in range (keylength):
        if cell1.key[keyindex] < cell2.key[keyindex]:
            return True
        elif cell1.key[keyindex] > cell2.key[keyindex]:
            return False
    return False


def testheap():
    global heap
    global heapsize

    for d in range(heapsize/2, 0, -1):
        if not keyless(heap[2*d],heap[d]):
             rospy.signal_shutdown()
        if 2*d+1 <= heapsize:
            if not keyless(heap[2*d+1],heap[d]):
                rospy.signal_shutdown()


def percolatedown(hole, tmpcell):
    global heap
    global heapsize

    if heapsize != 0:
        while 2*hole <= heapsize:
            child = 2*hole
            if child != heapsize and keyless(heap[child+1], heap[child]):
                child = child + 1
            if (keyless(heap[child], tmpcell)):
                heap[hole] = heap[child]
                heap[hole].heapindex = hole
            else:
                break
            hole = child

        heap[hole] = tmpcell
        heap[hole].heapindex = hole


def percolateup(hole, tmpcell):
    global heap
    global heapsize

    if heapsize != 0:
        while hole > 1 and keyless(tmpcell, heap[hole/2]):
            heap[hole] = heap[hole/2]
            heap[hole].heapindex = hole
            hole = hole/2
        heap[hole] = tmpcell
        heap[hole].heapindex = hole


def percolateupordown(hole, tmpcell):
    global heap
    global heapsize

    if heapsize != 0:
        if hole > 1 and keyless(tmpcell, heap[hole/2]):
            percolateup(hole, tmpcell)
        else:
            percolatedown(hole, tmpcell)


def emptyheap(length):
    global keylength
    global heapsize

    keylength = length
    heapsize = 0


def topheap():
    global heap
    global heapsize

    if heapsize == 0:
        return None
    return heap[1]


def deleteheap(thiscell):
    global heap
    global heapsize

    if thiscell.heapindex != 0 and thiscell.generated == initEnv.mazeiteration:
        percolateupordown(thiscell.heapindex, heap[heapsize])
        heapsize = heapsize - 1
        thiscell.heapindex = 0


def popheap():
    global heap
    global heapsize

    if (heapsize == 0):
	    return None
    thiscell = heap[1]
    thiscell.heapindex = 0
    percolatedown(1, heap[heapsize])
    heapsize = heapsize - 1
    return thiscell


def insertheap(thiscell):
    global heap
    global heapsize

    if thiscell.heapindex == 0 or thiscell.generated != initEnv.mazeiteration:
        thiscell.heapindex = 0
        # if DEBUG:
        #     if heapsize < HEAPSIZE-1:
        #         rospy.signal_shutdown()
        heapsize = heapsize + 1
        percolateup(heapsize, thiscell)
    else:
	    percolateupordown(thiscell.heapindex, heap[thiscell.heapindex])


#################################################################################
heap = {0:None}
heapsize = 0
keylength = 3
