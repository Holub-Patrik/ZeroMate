#include <hal/intdef.h>

extern "C" void __cxa_pure_virtual()
{
    while (1)
        ;
}

void* operator new(unsigned int)
{
    return (void*)0;
}

void* operator new[](unsigned int)
{
    return (void*)0;
}

void operator delete(void*)
{
}

void operator delete[](void*)
{
}

void operator delete(void*, unsigned int)
{
}

void operator delete[](void*, unsigned int)
{
}

extern "C" void __aeabi_unwind_cpp_pr0()
{
}

extern "C" void __aeabi_unwind_cpp_pr1()
{
}
