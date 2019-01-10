#include <stdint.h>
#include <stdio.h>

typedef int16_t  dctcoef;

typedef struct
{
    uint8_t i_bits;
    uint8_t i_size;
} vlc_t;

typedef struct
{
    uint16_t i_bits;
    uint8_t  i_size;
    /* Next level table to use */
    uint8_t  i_next;
} vlc_large_t;

typedef struct
{
    int32_t last;
    int32_t mask;
    dctcoef level[18];
} x264_run_level_t;

#define X264_MIN(a,b) ( (a)<(b) ? (a) : (b) )
#define LEVEL_TABLE_SIZE 128

vlc_large_t x264_level_token[7][LEVEL_TABLE_SIZE];
uint32_t x264_run_before[1<<16];

const vlc_t x264_run_before_init[7][16] =
{
    { /* i_zero_left 1 */
        { 0x1, 1 }, /* str=1 */
        { 0x0, 1 }, /* str=0 */
    },
    { /* i_zero_left 2 */
        { 0x1, 1 }, /* str=1 */
        { 0x1, 2 }, /* str=01 */
        { 0x0, 2 }, /* str=00 */
    },
    { /* i_zero_left 3 */
        { 0x3, 2 }, /* str=11 */
        { 0x2, 2 }, /* str=10 */
        { 0x1, 2 }, /* str=01 */
        { 0x0, 2 }, /* str=00 */
    },
    { /* i_zero_left 4 */
        { 0x3, 2 }, /* str=11 */
        { 0x2, 2 }, /* str=10 */
        { 0x1, 2 }, /* str=01 */
        { 0x1, 3 }, /* str=001 */
        { 0x0, 3 }, /* str=000 */
    },
    { /* i_zero_left 5 */
        { 0x3, 2 }, /* str=11 */
        { 0x2, 2 }, /* str=10 */
        { 0x3, 3 }, /* str=011 */
        { 0x2, 3 }, /* str=010 */
        { 0x1, 3 }, /* str=001 */
        { 0x0, 3 }, /* str=000 */
    },
    { /* i_zero_left 6 */
        { 0x3, 2 }, /* str=11 */
        { 0x0, 3 }, /* str=000 */
        { 0x1, 3 }, /* str=001 */
        { 0x3, 3 }, /* str=011 */
        { 0x2, 3 }, /* str=010 */
        { 0x5, 3 }, /* str=101 */
        { 0x4, 3 }, /* str=100 */
    },
    { /* i_zero_left >6 */
        { 0x7, 3 }, /* str=111 */
        { 0x6, 3 }, /* str=110 */
        { 0x5, 3 }, /* str=101 */
        { 0x4, 3 }, /* str=100 */
        { 0x3, 3 }, /* str=011 */
        { 0x2, 3 }, /* str=010 */
        { 0x1, 3 }, /* str=001 */
        { 0x1, 4 }, /* str=0001 */
        { 0x1, 5 }, /* str=00001 */
        { 0x1, 6 }, /* str=000001 */
        { 0x1, 7 }, /* str=0000001 */
        { 0x1, 8 }, /* str=00000001 */
        { 0x1, 9 }, /* str=000000001 */
        { 0x1, 10 }, /* str=0000000001 */
        { 0x1, 11 }, /* str=00000000001 */
    },
};

static int coeff_last16( dctcoef *l )
{
    int i_last = 15;
    while( i_last >= 0 && l[i_last] == 0 )
        i_last--;
    return i_last;
}

static int coeff_level_run16( dctcoef *dct, x264_run_level_t *runlevel )
{
    int i_last = runlevel->last = coeff_last16(dct);
    int i_total = 0;
    int mask = 0;
    do
    {
        runlevel->level[i_total++] = dct[i_last];
        mask |= 1 << (i_last);
        while( --i_last >= 0 && dct[i_last] == 0 );
    } while( i_last >= 0 );
    runlevel->mask = mask;
    return i_total;
}

static int x264_clz( uint32_t x )
{
    static uint8_t lut[16] = {4,3,2,2,1,1,1,1,0,0,0,0,0,0,0,0};
    int y, z = (((x >> 16) - 1) >> 27) & 16;
    x >>= z^16;
    z += y = ((x - 0x100) >> 28) & 8;
    x >>= y^8;
    z += y = ((x - 0x10) >> 29) & 4;
    x >>= y^4;
    return z + lut[x];
}

void x264_cavlc_init()
{
    for( int i_suffix = 0; i_suffix < 7; i_suffix++ )
        for( int16_t level = -LEVEL_TABLE_SIZE/2; level < LEVEL_TABLE_SIZE/2; level++ )
        {
            int mask = level >> 15;
            int abs_level = (level^mask)-mask;
            int i_level_code = abs_level*2-mask-2;
            int i_next = i_suffix;
            vlc_large_t *vlc = &x264_level_token[i_suffix][level+LEVEL_TABLE_SIZE/2];

            if( ( i_level_code >> i_suffix ) < 14 )
            {
                vlc->i_size = (i_level_code >> i_suffix) + 1 + i_suffix;
                vlc->i_bits = (1<<i_suffix) + (i_level_code & ((1<<i_suffix)-1));
            }
            else if( i_suffix == 0 && i_level_code < 30 )
            {
                vlc->i_size = 19;
                vlc->i_bits = (1<<4) + (i_level_code - 14);
            }
            else if( i_suffix > 0 && ( i_level_code >> i_suffix ) == 14 )
            {
                vlc->i_size = 15 + i_suffix;
                vlc->i_bits = (1<<i_suffix) + (i_level_code & ((1<<i_suffix)-1));
            }
            else
            {
                i_level_code -= 15 << i_suffix;
                if( i_suffix == 0 )
                    i_level_code -= 15;
                vlc->i_size = 28;
                vlc->i_bits = (1<<12) + i_level_code;
            }
            if( i_next == 0 )
                i_next++;
            if( abs_level > (3 << (i_next-1)) && i_next < 6 )
                i_next++;
            vlc->i_next = i_next;
        }

    x264_run_before[0] = 0;
    x264_run_before[1] = 0;
    for( uint32_t i = 2; i < (1<<16); i++ )
    {
        x264_run_level_t runlevel;
        dctcoef dct[16];
        int size = 0;
        int bits = 0;
        for( int j = 0; j < 16; j++ )
            dct[j] = i&(1<<j);
        int total = coeff_level_run16( dct, &runlevel );
        int zeros = runlevel.last + 1 - total;
        uint32_t mask = i << (x264_clz( i ) + 1);
        for( int j = 0; j < total-1 && zeros > 0; j++ )
        {
            int idx = X264_MIN(zeros, 7) - 1;
            int run = x264_clz( mask );
            int len = x264_run_before_init[idx][run].i_size;
            size += len;
            bits <<= len;
            bits |= x264_run_before_init[idx][run].i_bits;
            zeros -= run;
            mask <<= run + 1;
        }
        x264_run_before[i] = (bits << 5) + size;
    }
}

int main() {
    x264_cavlc_init();

    int i, j;
    printf("vlc_large_t x264_level_token[7][LEVEL_TABLE_SIZE] = {\n");
    for (i=0; i<7; i++) {
        printf("    {");
        for (j=0; j < LEVEL_TABLE_SIZE; j++) {
            if (j % 4 == 0)
                printf("\n       ");
            vlc_large_t *v = &x264_level_token[i][j];
            printf(" {%5d, %3d, %3d},", v->i_bits, v->i_size, v->i_next);
        }
        printf("\n    },\n");
    }
    printf("};\n\n");

    printf("uint32_t x264_run_before[1<<16] = {");
    for (i=0; i < (1<<16); i++) {
        if (i % 8 == 0) {
            printf("\n   ");
        }
        printf(" %9d,", x264_run_before[i]);
    }
    printf("\n};\n\n");

    return 0;
}
