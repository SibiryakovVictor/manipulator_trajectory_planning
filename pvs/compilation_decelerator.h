// This file is used to slow compilation down
// It doesn't have #pragma once deliberately

#if PVS_GENERATE_DUMP == 1

#include <stdint.h>

namespace {

    constexpr float power( float  n, uint32_t pow )
    {
        return ( ( pow != 0 ) ? ( n * power( n, pow - 1 ) ) : 1 );
    }

    constexpr uint32_t factorial( uint32_t n )
    {
        return ( ( n != 0 ) ? ( n * factorial( n - 1 ) ) : 1 );
    }

    //слагаемое в ряде Тейлора
    constexpr float sin_summand( float angle, uint32_t n )
    {
        return power( -1, n ) * power( angle, 2*n+1 ) / factorial(2*n+1);
    }

    //  сумма слагаемых ряда Тейлора
    constexpr float sinus_ce( float angle, uint32_t step )
    {
        return ( (step) ? ( sin_summand( angle, step ) + sinus_ce( angle, step - 1 ) ) : ( sin_summand( angle, step ) ) );
    }

    // структура для заполнения таблицы синусов внутри себя
    // общий случай
    template <uint32_t N, uint32_t X, uint32_t TNumberOfPointsTaylor>
    struct gen_sinus
    {
        template <uint32_t... vals>
        struct generated : gen_sinus<N-1, X, TNumberOfPointsTaylor>::template generated<N-1, vals ...>
        {
        };
    };

    // начало таблицы
    template <uint32_t X, uint32_t TNumberOfPointsTaylor>
    struct gen_sinus<X, X, TNumberOfPointsTaylor>
    {
        struct generated : gen_sinus<X-1, X, TNumberOfPointsTaylor>::template generated<X-1>
        {
        };
    };

    // перевод из номера элемента в таблице во флоат, таблица только на четверть синуса!
    template<uint32_t TLen>
    constexpr float convertToRad( uint32_t A )
    {
        return 1.5708f * A / ( TLen - 1 );
    }

    // перевод из флоата в номер точки в табличном синусе
    template<uint32_t TLen>
    constexpr uint32_t convertFromRad( float A )
    {
        return A * ( TLen ) / 6.2831853f;
    }

    //конец таблицы, получаем массив статик полем структуры
    template <uint32_t X, uint32_t TNumberOfPointsTaylor>
    struct gen_sinus<0, X, TNumberOfPointsTaylor>
    {
        template <uint32_t... vals>
        struct generated
        {
            static constexpr float values[X] = { sinus_ce( convertToRad<X>( vals ), TNumberOfPointsTaylor )... };
        };
    };

    template <uint32_t X, uint32_t TNumberOfPointsTaylor>
    template <uint32_t... vals>
    constexpr float gen_sinus<0, X, TNumberOfPointsTaylor>::generated<vals ...>::values[X];
   
    template<uint32_t TPoinsNum, uint32_t TNumberOfPointsTaylor>
    float staticSinus( uint32_t angle )
    {
        static const uint32_t points_in_table = TPoinsNum/4;
        if( angle <= points_in_table )
        {
            return gen_sinus<points_in_table+1, points_in_table+1, TNumberOfPointsTaylor>::generated::values[angle];
        }
        if(angle <= 2 * points_in_table )
        {
            return gen_sinus<points_in_table+1, points_in_table+1, TNumberOfPointsTaylor>::generated::values[( 2 * points_in_table ) - angle];
        }
        if(angle <= 3 * points_in_table )
        {
            return -gen_sinus<points_in_table+1, points_in_table+1, TNumberOfPointsTaylor>::generated::values[angle - ( 2 * points_in_table )];
        }
        if(angle <= 4 * points_in_table)
        {
            return -gen_sinus<points_in_table+1, points_in_table+1, TNumberOfPointsTaylor>::generated::values[4 * points_in_table - angle];
        }
        return 0;

    }

// -V::674
static const float sin_15 = staticSinus<500, 5>( 0.2618f );

}

#endif
