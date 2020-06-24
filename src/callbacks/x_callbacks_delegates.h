/***************************************************************************************************

 Это служебный файл, не подключайте его напрямую; не используйте классы отсюда напрямую!

***************************************************************************************************/

#pragma once

#include <new>
#include <limits>
#include <cstddef>
#include <stdint.h>

// отладочный вывод
#ifdef CALLBACK_DEBUG_ENABLE
    #include <stdio.h>

    #define CALLBACK_DEBUG_PRINTF     printf
#else
    #define CALLBACK_DEBUG_PRINTF( ... )
#endif

namespace callback { namespace additional
{
        // ------------------------ is_int
        //эксплиситный конструктор есть только от инта, так что и проверку делаю только на этот тип
        template<class T>
        struct is_int {typedef T type;};

        template<>
        struct is_int<int> {  };

        // ------------------------ структура для проверки, является ли метод константным
        template< class T >
        struct IsMethodConst;

        // случай константности
        template< class TObj, class TOut, class ... TIn >
        struct IsMethodConst < TOut (TObj::*)( TIn ... ) const >
        {
            static const bool value = true;
        };
        // случай неконстантности
        template< class TObj, class TOut, class ... TIn >
        struct IsMethodConst < TOut (TObj::*)( TIn ... ) >
        {
            static const bool value = false;
        };

        // ------------------------ remove_reference - копипаста из type_traits

        template< class T > struct remove_reference        { typedef T type; };
        template< class T > struct remove_reference< T& >  { typedef T type; };
        template< class T > struct remove_reference< T&& > { typedef T type; };


    // енум для возможности использования виртуального оператора сравнения делегатов
     enum class DelegateType{ CONST_MEMBER,
                              MEMBER,
                              FUNCTOR,
                              LAMBDA,
                              FUNCTION };

     //В классе Callback лежит указатель на интерфейс функционального объекта IDelegate
     //От этого интерфейса наследуются все реализации делегатов для разного рода функциональных объектов
     //Позволяет биндить Callback с одинаковой сигнатурой к разного рода функциональным объектам
     template< typename TOutput, typename ... TInput >
     class IDelegate
     {

     public:

         virtual TOutput operator() ( TInput ... tInputs ) = 0;

         virtual IDelegate * clone( void * ) = 0;

         virtual bool operator==( IDelegate * that ) = 0;

         virtual DelegateType getType( void ) const = 0;

         virtual ~IDelegate(){}

     };

     // делегат для произвольного метода класса
     template< bool TIsMethodConst, class TObject, typename TMethod, typename TOutput, typename ... TInput>
     class DelegateMethod;


     // делегат для константного метода - хранит указатель на метод
     template< class TObject, typename TMethod, typename TOutput, typename ... TInput >
     class DelegateMethod< true, TObject, TMethod, TOutput, TInput...> : public IDelegate< TOutput, TInput ...>
     {

     public:

         DelegateMethod( const TObject & object, TMethod method ) :
             m_object( object ),
             m_method( method )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateMethod( const TObject & object, TMethod method )\n");
         }

         DelegateMethod( const DelegateMethod & that ) :
             m_object( that.m_object ),
             m_method( that.m_method )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateMethod( const DelegateMethod & that )\n");
         }

         // нехорошо плодить делегатов просто так, надо делать это клоном
         DelegateMethod & operator=( const DelegateMethod & that ) = delete;

         virtual TOutput operator() ( TInput ... tInputs ) override
         {
             return ( m_object.*m_method )( tInputs ... );
         }

         using BaseType = IDelegate< TOutput, TInput ... >;

         virtual BaseType * clone( void * place ) override
         {
             CALLBACK_DEBUG_PRINTF("Called: virtual BaseType * clone( void * place )\n");

             BaseType * ret = ( BaseType * )new( place ) DelegateMethod( *this );
             return ret;
         }

         virtual bool operator==( BaseType * that ) override
         {
             //если тип совпадает, то можно кастовать к нему
             if( that->getType() == getType() )
             {
                 return operator==( ( DelegateMethod * )that );
             }
             return false;
         }

         bool operator==( DelegateMethod * that )
         {
             return ( ( &m_object == &that->m_object ) && ( m_method == that->m_method ) );
         }

         virtual DelegateType getType( void ) const override
         {
             return DelegateType::CONST_MEMBER;
         }

         virtual ~DelegateMethod()
         {}

     private:

         const TObject & m_object;
         TMethod m_method;
     };

     // делегат для неконстантного метода - хранит ссылку на объект и указатель на метод
     template< class TObject, typename TMethod, typename TOutput, typename ... TInput >
     class DelegateMethod< false, TObject, TMethod, TOutput, TInput...> : public IDelegate< TOutput, TInput ...>
     {

     public:

         DelegateMethod( TObject & object, TMethod method ) :
             m_object( object ),
             m_method( method )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateMethod( TObject & object, TMethod method )\n");
         }

         DelegateMethod( const DelegateMethod & that ) :
             m_object( that.m_object ),
             m_method( that.m_method )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateMethod( const DelegateMethod & that )\n");
         }

         //нехорошо плодить делегатов просто так, надо делать это клоном
         DelegateMethod & operator=( const DelegateMethod & that ) = delete;


         virtual TOutput operator() ( TInput ... tInputs ) override
         {
             return ( m_object.*m_method )( tInputs ... );
         }


         using BaseType = IDelegate<TOutput, TInput ...>;

         virtual BaseType * clone( void * place ) override
         {
             BaseType * ret = ( BaseType * )new( place ) DelegateMethod( *this );
             return ret;
         }


         virtual bool operator==( BaseType * that ) override
         {
             //если тип совпадает, то можно кастовать к нему
             if( that->getType() == getType() )
             {
                 return operator==( ( DelegateMethod * )that );
             }
             return false;
         }

         bool operator==( DelegateMethod * that )
         {
             return ( ( &m_object == &that->m_object) && ( m_method == that->m_method ) );
         }
         virtual DelegateType getType( void ) const override
         {
             return DelegateType::MEMBER;
         }

         virtual ~DelegateMethod()
         {}


     private:

         TObject & m_object;

         TMethod m_method;
     };


     //делегат для оператора ()
     template< class TObject, typename TOutput, typename ... TInput >
     class DelegateLambda : public IDelegate < TOutput, TInput ... >
     {

     public:

         DelegateLambda( TObject && object ) : m_object( object )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateLambda( TObject && object ) : m_object( object )\n");
         }
         DelegateLambda( const DelegateLambda & that ) : m_object( that.m_object )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateLambda( const DelegateLambda & that ) : m_object( that.m_object )\n");
         }

         //нехорошо плодить делегатов просто так, надо делать это клоном
         DelegateLambda & operator=( const DelegateLambda & ) = delete;

         virtual TOutput operator() ( TInput ... tInputs ) override
         {
             return ( m_object )( tInputs ... );
         }

         using BaseType = IDelegate<TOutput, TInput ...>;

         virtual BaseType * clone( void * place ) override
         {
             BaseType * ret = ( BaseType * )new( place ) DelegateLambda( *this );
             return ret;
         }

         virtual bool operator==( BaseType * that ) override
         {
             // если тип совпадает, то можно кастовать к нему
             if( that->getType() == getType() )
             {
                 return operator==( ( DelegateLambda * )that );
             }
             return false;
         }

         // лямбды никогда друг другу не равны
         bool operator==( DelegateLambda * )
         {
             return false;
         }

         virtual DelegateType getType( void ) const override
         {
             return DelegateType::LAMBDA;
         }

         virtual ~DelegateLambda()
         {}

     private:

         typename additional::remove_reference<TObject>::type m_object;

     };

     // делегат для указателя на свободную функцию или статического метода
     template< typename TOutput, typename ... TInput >
     class DelegateFunction : public IDelegate < TOutput, TInput ... >
     {

     public:

         DelegateFunction( TOutput ( *function )( TInput ... ) ) : m_function( function )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateFunction( TOutput ( *function )( TInput ... ) ) : m_function( function )\n");
         }

         DelegateFunction( const DelegateFunction & that ) : m_function( that.m_function )
         {
             CALLBACK_DEBUG_PRINTF("Called: DelegateFunction( const DelegateFunction & that ) : m_function( that.m_function )\n");
         }

         //нехорошо плодить делегатов просто так, надо делать это клоном
         DelegateFunction & operator=( const DelegateFunction & that ) = delete;

         virtual TOutput operator() ( TInput ... tInputs ) override
         {
             return ( *m_function )( tInputs ... );
         }

         using BaseType = IDelegate<TOutput, TInput ...>;

         virtual BaseType * clone( void * place ) override
         {
             BaseType * ret = ( BaseType * )new( place ) DelegateFunction( *this );
             return ret;
         }

         virtual bool operator==( BaseType * that ) override
         {
             //если тип совпадает, то можно кастовать к нему
             if( that->getType() == getType() )
             {
                 return operator==( ( DelegateFunction * )that );
             }
             return false;
         }

         bool operator==( DelegateFunction * that )
         {
             return ( m_function == that->m_function );
         }

         virtual DelegateType getType( void ) const override
         {
             return DelegateType::FUNCTION;
         }

         virtual ~DelegateFunction()
         {}

     private:

         TOutput ( *m_function )( TInput ... );
     };

} } // namespace callback { namespace additional



