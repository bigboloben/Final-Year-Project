#ifndef IF_ZERO
#define IF_ZERO

void IfZeroReturnZero_float(float ATR, float ABL, float ATL, float ABR, out float Out)
{
#if (ATR != 0 || ABL != 0 || ATL != 0 || ABR != 0)
    Out = 1;
#else
    Out = 0;
#endif
}

#endif