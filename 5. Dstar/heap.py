import include
import maze

HEAPSIZE = 100000
cell *heap[HEAPSIZE]
int heapsize
int keylength

void emptyheap(int length)
int testheap()
cell* popheap()
cell *topheap()
void deleteheap(cell *thiscell)
void insertheap(cell *thiscell)


'''heap.c'''
cell *heap[HEAPSIZE]
heapsize = 0
keylength = 3

def keyless(cell *cell1, cell* cell2):
    for keyindex in range (keylength):
        if (cell1->key[keyindex] < cell2->key[keyindex]):
            return 1
        elif (cell1->key[keyindex] > cell2->key[keyindex]):
            return 0
    return 0


def testheap():
    for d in range(heapsize/2, 0, -1):
        assert(!keyless(heap[2*d],heap[d]))
        if (2*d+1 <= heapsize):
            assert(!keyless(heap[2*d+1],heap[d]))


def percolatedown(hole, cell *tmp):

    if heapsize != 0:
        while(2*hole <= heapsize):
            child = 2*hole
            if (child != heapsize && keyless(heap[child+1], heap[child])):
                ++child
            if (keyless(heap[child], tmp)):
                heap[hole] = heap[child]
                heap[hole]->heapindex = hole
            else:
                break
            hole = child

        heap[hole] = tmp
        heap[hole]->heapindex = hole


def percolateup(int hole, cell *tmp):
    if (heapsize != 0):
        while(hole > 1 && keyless(tmp, heap[hole/2])):
            heap[hole] = heap[hole/2]
            heap[hole]->heapindex = hole
            hole /= 2
        heap[hole] = tmp
        heap[hole]->heapindex = hole


def percolateupordown(int hole, cell *tmp):
    if (heapsize != 0):
        if (hole > 1 && keyless(tmp, heap[hole/2])):
            percolateup(hole, tmp)
        else:
            percolatedown(hole, tmp)

def emptyheap(int length):
    keylength = length
    heapsize = 0

def topheap():
    if (heapsize == 0):
	    return NULL
    return heap[1]


def deleteheap(cell *thiscell):
    if (thiscell->heapindex != 0 && thiscell->generated == mazeiteration):
        percolateupordown(thiscell->heapindex, heap[heapsize--])
        thiscell->heapindex = 0


def popheap():
    cell *thiscell

    if (heapsize == 0):
	    return NULL
    thiscell = heap[1]
    thiscell->heapindex = 0
    percolatedown(1, heap[heapsize--])
    return thiscell


def insertheap(cell *thiscell):
    int hole

    if (thiscell->heapindex == 0 || thiscell->generated != mazeiteration):
        thiscell->heapindex = 0

        if DEBUG:
            assert(heapsize < HEAPSIZE-1)

        percolateup(++heapsize, thiscell)
    else:
	    percolateupordown(thiscell->heapindex, heap[thiscell->heapindex])
