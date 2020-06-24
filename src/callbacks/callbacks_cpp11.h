/***************************************************************************************************

 Это велосипедная реализация std::function из С++11 - универсальные коллбэки.
 Они позволяют хранить указатель на свободную функцию, на метод класса с указателем на объект или 
 лямбду и вызывать их единообразно.

 При создании по-умолчанию инициализируются нулем. Их можно сравнивать друг с другом, сравнивать с 
 нулем. Можно явно инициализировать нулем.

 WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 Осознавайте, что делаете! Если вы создаете коллбек на метод у объекта, то объект ДОЛЖЕН дожить 
 до момента вызова коллбека!

 Лямбды и функторы всегда копируются при захвате - если вы не хотите, чтобы функтор копировался, 
 удалите его конструктор копирования (и получите ошибку компиляции :).
 
 Если вам хочется коллбек для функтора без копирования функтора - сделайте лямбду, которая 
 дергает функтор, получая его по ссылке - и создавайте коллбек из лямбды.

 Если при захвате лямбды или функтора вы получаете static_assert - функтор/лямбда слишком толстые.
 Попробуйте посадить их на диету или переопределить размер коллбека с помощью 
 CALLBACK_USER_DEFINED_SIZE.
 
 Это крайняя мера для крайних случаев! Объявлять его нужно на весь проект, чтобы не получить в
 разных местах коллбеки разного размера (и кучу проблем).

 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING

 Пример использования:

 callback::Callback<void (int)> c; // создал экземпляр коллбэка, который возвращает void и принимает
                                   // 1 параметр типа int

 c = CALLBACK_BIND( a, A::foo );  // прибиндил к нему метод foo объекта а класса А

 c = CALLBACK_BIND( bar );        // прибиндил к нему свободную функцию bar

 c = CALLBACK_BIND( A::bazz );    // прибиндил к нему статический метод класса А

 c = CALLBACK_BIND( [data](int a){ std::printf("lambda\n");} ); // прибиндил к нему лямбду

 c = CALLBACK_BIND( a );        // прибиндил к нему функтор а ( класс с перегруженным оператором () )


Помимо макросов можно писать так:

callback::Callback<void ( int )> c( bar ); //прибиндил свободную функцию в конструкторе

callback::Callback<void ( int )> c( A::bazz ); //прибиндил статический метод класса А в конструкторе

callback::Callback<void ( int )> c( [data](int a){ std::printf("lambda\n");} ); //прибиндил лямбду в конструкторе

callback::Callback<void ( int )> c( a ); //прибиндил функтор в конструкторе


Или вот так:

c = bar; //прибиндил свободную функцию

c = A::bazz; //прибиндил статический метод класса А

c = [data](int a){ std::printf("lambda\n");}; //прибиндил лямбду

c = a; //прибиндил функтор


Теперь у коллбеков есть конструкторы от callable-объектов, что даёт возможность делать следующее:

void foo( Callback<int (int)> c );

int bar(int);

int main
{
    foo(bar); // передаём в функцию объект Callback, к которому был прибинден bar
}

Это работает с функторами, свободными функциями, статическими методами и лямбдами

 c(5);                            // вызов прибинденного



можно еще вот так биндить, если хочется:

c.bind<bar>(); //прибиндил свободную функцию

c.bind<A::bazz>(); //прибиндил статический метод класса А

c.bind( [data](int a){ std::printf("lambda\n");} ); //прибиндил лямбду

c.bind(a); //прибиндил функтор

c.bind(a, &A::foo2); //прибиндил нестатический метод класса

 Апи перетащено с коллбеков прежних.
***************************************************************************************************/


#pragma once

#include <new>
#include <limits>
#include <cstddef>
#include <stdint.h>

#include "x_callbacks_delegates.h"

// макросы почти без изменений переехали
// это служебный макрос для перегрузки
#define CALLBACK_GET_BIND_MACRO( _1, _2, NAME, ... )  NAME

// это универсальный макрос для биндинга, он принимает или имя свободной функции или имя объекта+имя метода
// DUMMY нужен, чтобы убрать ворнинг про "ISO C++11 requires at least one argument for the "..." in a variadic macro"
#define CALLBACK_BIND( ... ) CALLBACK_GET_BIND_MACRO( __VA_ARGS__, CALLBACK_BIND_MEMBER, CALLBACK_BIND_FREE, DUMMY ) (__VA_ARGS__)

// это макрос для биндинга метода
#define CALLBACK_BIND_MEMBER( object, member ) ( callback::makeMemberCallback(&member)\
                                                .bind( object, &member ) )

// это макрос для биндинга остального
#define CALLBACK_BIND_FREE( stuff )  ( stuff )



// максимальный допустимый размер экземпляра коллбека

// он должен мочь хранить в себе указатель на функцию, на метод, на функтор и копию экземпляра лямбды,
// если у лямбды есть захват

// если вы ловите static_assert'ы про нехватку места, сделайте эту константу побольше
// но поскольку эти коллбеки не выделяют память в куче, они ВСЕ таскают в себе место с запасом!
// поэтому будьте осторожны - ВСЕ экземпляры коллбеков станут толще!

// по умолчанию выбирается максимальный размер указателя среди функции, метода и функтора
// на Cortex-M обычно получается 16 байт

#ifndef CALLBACK_USER_DEFINED_SIZE
    #define CALLBACK_USER_DEFINED_SIZE    0
#endif

namespace callback
{
//    template
//    using IDelegate        = additional::IDelegate;
//
//    using DelegateFunction = additional::DelegateFunction;
//    using DelegateLambda   = additional::DelegateLambda;
//    using DelegateMethod   = additional::DelegateMethod;
//    using DelegateType     = additional::DelegateType;

    //пустая структура, можно сунуть в конструктор коллбеку. Осталось для обратной совместимости
    struct NullCallback {};
    
    //класс, экземпляры которого надо использовать для унификации объектов типа callable
    template <class T>
    class Callback;
    
    //функции для создания локального экземпляра коллбека, чтобы CALLBACK_BIND реализовать обратносовместимо
    //создаёт экземпляр коллбека нужного типа
    template< class TObject, typename TOutput, typename ... TInput >
    Callback < TOutput( TInput ... ) > makeMemberCallback( TOutput (TObject::*)( TInput ... ) )
    {
        Callback<TOutput ( TInput ... )> callable;
        return callable;
    }

    //функции для создания локального экземпляра коллбека, чтобы CALLBACK_BIND реализовать обратносовместимо
    template< class TObject, typename TOutput, typename ... TInput >
    Callback < TOutput( TInput ... ) > makeMemberCallback( TOutput (TObject::*)( TInput ... ) const )
    {
        Callback<TOutput ( TInput ... )> callable;
        return callable;
    }
    

    //реализация класса универсального коллбека
    template< typename TOutput, typename ... TInput >
    class Callback < TOutput( TInput ... ) >
    {

    public:

        Callback( ){}

        explicit Callback( int )
        {
            CALLBACK_DEBUG_PRINTF("Called: explicit Callback( int a )\n");
        }

        Callback( decltype( nullptr ) )
        {
            CALLBACK_DEBUG_PRINTF("Called: Callback( decltype( nullptr ) a )\n");
        }

        Callback( NullCallback )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback( NullCallback )\n");
        }

        Callback( const Callback & rhs ) : m_delegate( rhs.m_delegate )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback( const Callback & rhs ) : m_delegate( rhs.m_delegate )\n" );

            *this = rhs;
        }

        Callback( Callback & rhs ) : m_delegate( rhs.m_delegate )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback( Callback & rhs ) : m_delegate( rhs.m_delegate )\n");

            *this = rhs;
        }

        // явный move-конструктор
        Callback( Callback && rhs ) : m_delegate( rhs.m_delegate )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback( Callback && rhs ) : m_delegate( rhs.m_delegate )\n");

            *this = rhs;
        }

        // явный const move-конструктор
        Callback( const Callback && rhs ) : m_delegate( rhs.m_delegate )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback( const Callback && rhs ) : m_delegate( rhs.m_delegate )\n");

            *this = rhs;
        }

        // есть опасность, что эти сверхшаблонные конструкторы заменят собой конструктор копирования
        // или перемещающий конструктор
        // enable_if'a, чтобы это полечить гарантированно, у нас пока нет,
        // поэтому пока что ограничимся явными конструкторами

        // конструктор для временных функторов, в т.ч. для лямбд
        template< class TObject, typename = typename additional::is_int < TObject >::type >
        Callback( TObject && object )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: temporary functor ctor -- "
                "template<class TObject, typename = typename additional::is_int<TObject>::type >"
                "Callback( TObject && object )\n");
        
            bind( (TObject &&)object );
        }

        // конструктор для свободной функции
        Callback( TOutput ( *fun )( TInput ... ) )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: free function ctor -- Callback( TOutput ( *fun )( TInput ... ) )\n");
        
            bind( fun );
        }

        Callback & operator=( NullCallback )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback & operator=( NullCallback )\n");
        
            m_delegate = nullptr;
            return *this;
        }

        Callback & operator=( const Callback & rhs )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback & operator=( const Callback & rhs )\n");
            
            if( rhs.m_delegate != nullptr )
            {
                m_delegate = rhs.m_delegate->clone( m_place );
            }
            return *this;
        }

        Callback & operator=( Callback & rhs )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: Callback & operator=( Callback & rhs )\n");
            
            if( rhs.m_delegate != nullptr )
            {
                m_delegate = rhs.m_delegate->clone( m_place );
            }
            return *this;
        }


        inline bool operator!() const
        {
            return m_delegate == nullptr;
        }

        operator bool() const
        {
            return !(m_delegate == nullptr);
        }

        TOutput operator() ( TInput ... tInputs ) const
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: TOutput operator() ( TInput ... tInputs ) const\n");
            
            return m_delegate->operator()( tInputs ... );
        }
    
        // бинд свободной функции
        Callback & bind( TOutput ( *function )( TInput ... ) )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: bind for free function -- "
                "Callback & bind( TOutput ( *function )( TInput ... ) )\n");
            
            using DelegateAlias = additional::DelegateFunction<TOutput, TInput ...>;

            static_assert( sizeof(m_place) >= sizeof( DelegateAlias ),
                           "m_place should be bigger than Delegate object" );

            m_delegate = new( m_place ) DelegateAlias( function );
            return *this;
        }

        // бинд для произвольного метода
        template< class TObject, typename TMethodPointer >
        Callback & bind( TObject & object, TMethodPointer method )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: bind for method -- template<class TObject, typename TMethodPointer> "
                "Callback & bind( TObject & object, TMethodPointer Tmethod )\n");
            
            using DelegateAlias =
                additional::DelegateMethod
                <
                    additional::IsMethodConst< TMethodPointer>::value,
                    TObject,
                    TMethodPointer,
                    TOutput,
                    TInput ...
                >;

            static_assert( sizeof(m_place) >= sizeof( DelegateAlias ),
                           "m_place should be bigger than Delegate object" );

            m_delegate = new( m_place ) DelegateAlias( object, method );
      
            return *this;
        }

        // универсальный бинд для функтора или лямбды - копирующий!
        template< typename TObject >
        Callback & bind( TObject && object )
        {
            CALLBACK_DEBUG_PRINTF(
                "Called: bind for universal reference -- template<typename TObject>"
                "Callback & bind( TObject && object )\n");

            using DelegateAlias = additional::DelegateLambda< TObject, TOutput, TInput ...>;
            
            static_assert( sizeof(m_place) >= sizeof( DelegateAlias ),
                           "m_place should be bigger than Delegate object.\n "
                           "You are probably created a lambda with big capture list "
                           "or a functor with too many fields.\n"
                           "If you really need this to work, you can define CALLBACK_USER_DEFINED_SIZE with a bigger value" );

            CALLBACK_DEBUG_PRINTF("delegate size = %d, object size = %d, m_place = %d\n",
                                   sizeof(DelegateAlias), sizeof(TObject), sizeof(m_place) );

            m_delegate = new( m_place ) DelegateAlias( (TObject &&) object );
            
            return *this;
        }

        bool operator==( const Callback & that )
        {
            return ( m_delegate->operator==( that.m_delegate ) );
        }

        bool operator!=( const Callback & that )
        {
            return !( operator==( that ) );
        }

        bool operator==( const int num )
        {
            if( num != 0 )
            {
                UMBA_ASSERT_FAIL();
            }
            
            if( m_delegate == nullptr ) 
            {
                return true;
            }
            
            return false;
        }

        bool operator!=( const int num )
        {
            return !( operator==( num ) );
        }


        ~Callback() {}

    private:

        class Dummy {};

        // поиск делегата наибольшего размера для плейсмент нью
        using Method   = additional::DelegateMethod< false, Callback, void (Callback::*)(void), void >;
        using Function = additional::DelegateFunction< void >;
        using Lambda   = additional::DelegateLambda< Dummy, int, int >;


        static const size_t method_size = sizeof( Method );
        static const size_t function_size = sizeof( Function );
        static const size_t lambda_size = sizeof( Lambda );

        static const size_t user_defined_size = CALLBACK_USER_DEFINED_SIZE;

        static const size_t max_size_tmp   = ( method_size  > user_defined_size ) ? method_size  : user_defined_size;
        static const size_t max_size_tmp_2 = ( max_size_tmp > function_size     ) ? max_size_tmp : function_size;

        static const size_t max_size = max_size_tmp_2;

        union
        {
            // по-идее этот атрибут дает максимальное выравнивание для родных типов
            char m_place[ max_size ] __attribute__((aligned));

            //поля, которые нужны для выравнивания в памяти массива байт под плейсмент нью
            Method never_used;
            Function never_used1;
            Lambda never_used2;
        };

        //указатель на базовый класс, в котором лежит адрес статического функтора
        additional::IDelegate < TOutput, TInput ... > * m_delegate = nullptr;
    };

    // коллбек без параметров
    using VoidCallback = Callback< void ( void ) >;
    
    
}//namespace callback
