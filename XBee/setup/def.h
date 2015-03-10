#ifndef DEF_HEADER
#define DEF_HEADER

typedef byte uint8_t;

template <typename T>
union bytes
{
    T f;
    byte b[sizeof(T)];
};

#endif
