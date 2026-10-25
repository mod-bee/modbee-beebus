#pragma once

// Included last by each library source file, after every other include.
//
// The library throws nothing and catches nothing, so its own functions need no
// exception tables or unwind code: about 4 KB of flash on a board. Placed after
// the standard headers so they keep their own settings; an application handler
// that throws through the library ends the program, which on a controller is
// what an escaped exception does anyway.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("no-exceptions")
#endif
