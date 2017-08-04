'''
/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */
/* Note: this version of D* Lite is optimized for grids                  */
/* It assumes, for example, that no cell can be a successor of itself.   */
'''
import include
import heap
import maze

int keymodifier;
cell goaltmpcell, oldtmpcell;

void printactualmaze(FILE *output)
{
    fprintf(output, "Actual maze: \n");
    int x, y;

    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n");
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
        fprintf(output, "X");
        for (x = 0; x < MAZEWIDTH; ++x)
        {
            if (mazegoal == &maze[y][x])
            fprintf(output, "R");
            else if (mazestart == &maze[y][x])
            fprintf(output, "G");
            else if (maze[y][x].obstacle)
            fprintf(output, "X");
            else
            fprintf(output, " ");
        }
        fprintf(output, "X\n");
    }
    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n\n\n");
}

void printknownmaze(FILE *output)
{
    int x, y, d;
    static char **display = NULL;
    cell *tmpcell;

    if (display == NULL)
    {
        display = (char **)calloc(MAZEHEIGHT, sizeof(char *));
        for (y = 0; y < MAZEHEIGHT; ++y)
            display[y] = (char *)calloc(MAZEWIDTH, sizeof(char));
    }
    for (y = 0; y < MAZEHEIGHT; ++y)
        for (x = 0; x < MAZEWIDTH; ++x)
        {
            display[y][x] = 'X';
            for (d = 0; d < DIRECTIONS; ++d)
                if (maze[y][x].move[d])
                    display[y][x] = ' ';
        }
    for (tmpcell = mazegoal; tmpcell != mazestart; tmpcell = tmpcell->searchtree)
	    display[tmpcell->y][tmpcell->x] = '.';
    display[mazestart->y][mazestart->x] = 'G';
    display[mazegoal->y][mazegoal->x] = 'R';
    for (x = 0; x < MAZEWIDTH+2; ++x)
	    fprintf(output, "X");
    fprintf(output, "\n");
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
        fprintf(output, "X");
        for (x = 0; x < MAZEWIDTH; ++x)
            fprintf(output, "%c", display[y][x]);
        fprintf(output, "X\n");
    }
    for (x = 0; x < MAZEWIDTH+2; ++x)
	    fprintf(output, "X");
    fprintf(output, "\n\n\n");
}

void initialize()
{
    ++mazeiteration;
    keymodifier = 0;
    mazestart->g = LARGE;
    mazestart->rhs = 0;
#ifdef TIEBREAKING
    emptyheap(3);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = H(mazestart) + 1;
    mazestart->key[2] = H(mazestart);
#else
    emptyheap(2);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = 0;
#endif
    mazestart->searchtree = NULL;
    mazestart->generated = mazeiteration;
    insertheap(mazestart);
    mazegoal->g = LARGE;
    mazegoal->rhs = LARGE;
    mazegoal->searchtree = NULL;
    mazegoal->generated = mazeiteration;
}

#ifdef RANDOMIZESUCCS
int permute[DIRECTIONS];
int* permutation[DIRECTIONS];
int permutations;

void swappermutations(int n)
{
    int i;
    int swap;

    if (n) // ??? don't need brackets?
	for (i = 0; i <= n; ++i)
	{
	    swappermutations(n-1);
	    if (n % 2)
	    {
            swap = permute[n];
            permute[n] = permute[i];
            permute[i] = swap;
	    }
	    else
	    {
            swap = permute[n];
            permute[n] = permute[0];
            permute[0] = swap;
        }
	}
    else
    {
        for (i = 0; i < DIRECTIONS; ++i)
            permutation[i][permutations] = permute[i];
        permutations++;
    }
}

void createpermutations()
{
    int i, j;

    permutations = 2;
    for (i = 3; i <= DIRECTIONS; ++i)
	    permutations *= i;
    for (i = 0; i < DIRECTIONS; ++i)
    {
        permute[i] = i;
        permutation[i] = calloc(permutations, sizeof(int));
    }
    permutations = 0;
    swappermutations(DIRECTIONS-1);
}
#endif

void initializecell(cell *thiscell)
{
    if (thiscell->generated != mazeiteration)
    {
        thiscell->g = LARGE;
        thiscell->rhs = LARGE;
        thiscell->searchtree = NULL;
        thiscell->generated = mazeiteration;
    }
}

void updatecell(cell *thiscell)
{
    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
        thiscell->key[2] = thiscell->g;
#else
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->g;
#endif
	    insertheap(thiscell);
    }
    else if (thiscell->g > thiscell->rhs)
    {
#ifdef TIEBREAKING
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
        thiscell->key[2] = H(thiscell) + keymodifier;
#else
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs;
#endif
	    insertheap(thiscell);
    }
    else
	    deleteheap(thiscell);
}

void updatekey(cell *thiscell)
{
    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
        thiscell->key[2] = thiscell->g;
#else
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
	    thiscell->key[1] = thiscell->g;
#endif
    }
    else
    {
#ifdef TIEBREAKING
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
        thiscell->key[2] = H(thiscell) + keymodifier;
#else
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs;
#endif
    }
}

void updaterhs(cell *thiscell)
{
    int d;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

    thiscell->rhs = LARGE;
    thiscell->searchtree = NULL;
#ifdef RANDOMIZESUCCS
    dcase = random() % permutations;
    for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
    {
        d = permutation[dtemp][dcase];
#else
    for (d = 0; d < DIRECTIONS; ++d)
    {
#endif
        if (thiscell->move[d] && thiscell->move[d]->generated == mazeiteration && thiscell->rhs > thiscell->move[d]->g + 1)
        {
            thiscell->rhs = thiscell->move[d]->g + 1;
            thiscell->searchtree = thiscell->move[d];
        }
    }
    updatecell(thiscell);
}

int computeshortestpath()
{
    cell *tmpcell1, *tmpcell2;
    int x, d;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

#ifdef TIEBREAKING
    if (mazegoal->g < mazegoal->rhs)
    {
        goaltmpcell.key[0] = mazegoal->g + keymodifier;
        goaltmpcell.key[1] = mazegoal->g + keymodifier;
        goaltmpcell.key[2] = mazegoal->g;
    }
    else
    {
        goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
        goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
        goaltmpcell.key[2] = keymodifier;
    }
#else
    if (mazegoal->g < mazegoal->rhs)
    {
        goaltmpcell.key[0] = mazegoal->g + keymodifier;
        goaltmpcell.key[1] = mazegoal->g;
    }
    else
    {
        goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
        goaltmpcell.key[1] = mazegoal->rhs;
    }
#endif
    while (topheap() && (mazegoal->rhs > mazegoal->g || keyless(topheap(), &goaltmpcell)))
    {
        tmpcell1 = topheap();
        oldtmpcell.key[0] = tmpcell1->key[0];
        oldtmpcell.key[1] = tmpcell1->key[1];
#ifdef TIEBREAKING
        oldtmpcell.key[2] = tmpcell1->key[2];
#endif
        updatekey(tmpcell1);
        if (keyless(&oldtmpcell, tmpcell1))
            updatecell(tmpcell1);
        else if (tmpcell1->g > tmpcell1->rhs)
        {
            tmpcell1->g = tmpcell1->rhs;
            deleteheap(tmpcell1);
#ifdef RANDOMIZESUCCS
            dcase = random() % permutations;
            for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
            {
            d = permutation[dtemp][dcase];
#else
            for (d = 0; d < DIRECTIONS; ++d)
            {
#endif
                if (tmpcell1->move[d])
                {
                    tmpcell2 = tmpcell1->move[d];
                    initializecell(tmpcell2);
                    if (tmpcell2 != mazestart && tmpcell2->rhs > tmpcell1->g + 1)
                    {
                    tmpcell2->rhs = tmpcell1->g + 1;
                    tmpcell2->searchtree = tmpcell1;
                    updatecell(tmpcell2);
                    }
		        }
	        }
	    }
        else
        {
            tmpcell1->g = LARGE;
            updatecell(tmpcell1);
#ifdef RANDOMIZESUCCS
            dcase = random() % permutations;
            for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
            {
              d = permutation[dtemp][dcase];
#else
            for (d = 0; d < DIRECTIONS; ++d)
            {
    #endif
                if (tmpcell1->move[d])
                {
                tmpcell2 = tmpcell1->move[d];
                initializecell(tmpcell2);
                if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1)
                  updaterhs(tmpcell2);
                }
            }
        }
#ifdef TIEBREAKING
        if (mazegoal->g < mazegoal->rhs)
        {
            goaltmpcell.key[0] = mazegoal->g + keymodifier;
            goaltmpcell.key[1] = mazegoal->g + keymodifier;
            goaltmpcell.key[2] = mazegoal->g;
        }
        else
        {
            goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
            goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
            goaltmpcell.key[2] = keymodifier;
        }
#else
        if (mazegoal->g < mazegoal->rhs)
        {
            goaltmpcell.key[0] = mazegoal->g + keymodifier;
            goaltmpcell.key[1] = mazegoal->g;
        }
        else
        {
            goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
            goaltmpcell.key[1] = mazegoal->rhs;
        }
#endif
    }
    return (mazegoal->rhs == LARGE);
}

void updatemaze(cell *robot)
{
    int d1, d2;
    cell *tmpcell;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

#ifdef RANDOMIZESUCCS
    dcase = random() % permutations;
    for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
    {
    d1 = permutation[dtemp][dcase];
#else
    for (d1 = 0; d1 < DIRECTIONS; ++d1)
    {
#endif
        if (robot->move[d1] && robot->move[d1]->obstacle)
        {
            tmpcell = robot->move[d1];
            initializecell(tmpcell);
            for (d2 = 0; d2 < DIRECTIONS; ++d2) // ??? d2 = DIRECTIONS-1 ???
            if (tmpcell->move[d2])
            {
                tmpcell->move[d2] = NULL;
                tmpcell->succ[d2]->move[reverse[d2]] = NULL;
                initializecell(tmpcell->succ[d2]);
                if (tmpcell->succ[d2] != mazestart && tmpcell->succ[d2]->searchtree == tmpcell)
                updaterhs(tmpcell->succ[d2]);
            }
            if (tmpcell != mazestart)
            {
                tmpcell->rhs = LARGE;
                updatecell(tmpcell);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    int k, l;
    cell *tmpcell;
    cell *lastcell;

#ifdef RANDOMIZESUCCS
    createpermutations();
#endif
    srandom(13);
    for (k = 0; k < RUNS; ++k)
    {
	    printf("maze %d\n", k);
#ifdef RANDOMMAZE
	    newrandommaze();
#else
	    newdfsmaze(WALLSTOREMOVE);
#endif
#ifdef DISPLAY
	    printactualmaze(stdout);
#endif
	    initialize();
	    fflush(stdout);
	    lastcell = mazegoal;
        while (mazestart != mazegoal)
        {
            if (computeshortestpath())
            break;
#ifdef DISPLAY
	        printknownmaze(stdout);
#endif
            mazegoal->trace = NULL;
            do
            {
                mazegoal->searchtree->trace = mazegoal;
                mazegoal = mazegoal->searchtree;
            } while (mazestart != mazegoal && !mazegoal->searchtree->obstacle);

            if (mazestart != mazegoal)
            {
                keymodifier += H(lastcell);
                lastcell = mazegoal;
                for (tmpcell=mazegoal; tmpcell; tmpcell=tmpcell->trace)
                    updatemaze(tmpcell);
            }
	    }
    }
}
