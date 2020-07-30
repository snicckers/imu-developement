#include <math.h>

// Cheapest/fastest inverse square root I could find (99.94% accurate to 1 / sqrt(x))
// Source: http://www.dbfinteractive.com/forum/index.php?topic=6269.0
float invSqrt( float x ){
    float xhalf = 0.5f*x;
    union {
        float x;
        int i;
    } u;
    u.x = x;
    u.i = 0x5f375a86 - (u.i >> 1);
    /* The next line can be repeated any number of times to increase accuracy */
    u.x = u.x * (1.5f - xhalf * u.x * u.x);
    return u.x;
}

// Source:
double FastSqrt(double x) {
    if (x <= 0)
        return 0;       // if negative number throw an exception?
    int exp = 0;
    x = frexp(x, &exp); // extract binary exponent from x
    if (exp & 1) {      // we want exponent to be even
        exp--;
        x *= 2;
    }
    double y = (1+x)/2; // first approximation
    double z = 0;
    while (y != z) {    // yes, we CAN compare doubles here!
        z = y;
        y = (y + x/y) / 2;
    }
    return ldexp(y, exp/2); // multiply answer by 2^(exp/2)
}

// #include <stdio.h>
// #include <math.h>
// #include <time.h>
//
// float fastsqrt( float n )
// {
// 	int i = *( int* )&n;
// 	i -= 1 << 23;
// 	i >>= 1;
// 	i += 1 << 29;
// 	return *( float* )&i;
// }
//
// double Sqrt2(double x) {
//     if (x <= 0)
//         return 0;       // if negative number throw an exception?
//     int exp = 0;
//     x = frexp(x, &exp); // extract binary exponent from x
//     if (exp & 1) {      // we want exponent to be even
//         exp--;
//         x *= 2;
//     }
//     double y = (1+x)/2; // first approximation
//     double z = 0;
//     while (y != z) {    // yes, we CAN compare doubles here!
//         z = y;
//         y = (y + x/y) / 2;
//     }
//     return ldexp(y, exp/2); // multiply answer by 2^(exp/2)
// }
//
// int main(void) {
//
//   clock_t t;
//   t = clock();
//
//
//   float x = 0.6f;
//
//   float a = fastsqrt(x);
//   t = clock() - t;
//   double time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
//   printf("%9.6f", a);
//   printf("%9.6f", time_taken);
//   printf("\n");
//
//
//   float b = sqrt(x);
//   t = clock() - t;
//   time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
//   printf("%9.6f", b);
//   printf("%9.6f", time_taken);
//   printf("\n");
//
//   float c = Sqrt2(x);
//   t = clock() - t;
//   time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
//   printf("%9.6f", c);
//   printf("%9.6f", time_taken);
//   printf("\n");
//
//   printf("Hello World\n");
//   return 0;
// }
