/***************************************************************************************************
                              Это файл, который затыкает собой щели.
 
 В нем лежит atexit, assert_failed, обвязка для printf'a и всякая веселуха
 
***************************************************************************************************/

#include "project_config.h"
#include "retarget.h"

/**************************************************************************************************
 Это мы так запихиваем по фиксированному адресу время компиляции и хеш коммита
 Строчка с define COMMIT_DATA генерируется скриптом update_commit_hash.bat, который нужно
 вызывать до билда. Чтобы от изменения этого файла не изменялась рабочая копия, после билда нужно
 вызывать скрипт remove_commit_hash.bat.
 
 Для этого в меню project->options->user нужно выбрать соответствующие скрипты (по относительному пути)
 в пунктах Before build/rebuild и After build/rebuild.
 
 Для использования этого скрипта нужен bash от гита. Чтобы помочь скрипту его найти, создайте
 переменную окружения GIT_BASH_PATH (95% что она будет равна "C:\Program Files (x86)\Git\usr\bin\" 
 
 Строчка текста с инфой будет лежать в памяти. Если вы хотите положить ее по какому-то определнному 
 адресу, создайте дефайн RETARGET_BUILD_INFO_ADDRESS, равный этому адресу.
 !!! Но будьте осторожны !!!! Размещая эту строчку по фиксированному адресу вы рискуете:
  - стереть таблицу векторов
  - сломать работу с загрузчиком; потому что этот фиксированный адрес игнорирует все остальные смещения

 Вся эта функциональность включена по-умолчанию. 
 Для отключения - создайте дефайн RETARGET_DISABLE_BUILD_INFO.
 
 Все дефайны можно класть в project_config.h.
 

**************************************************************************************************/

// эти дефайны можно выставлять в project_config, а не на весь проект
#ifndef UMBA_RETARGET_DISABLE_BUILD_INFO

    namespace umba { namespace retarget
    {

        // типовые костыли для слияния макроса со строкой
        // нужны для clang-аттрибута
        #define RETARGET_BUILD_INFO_ADDRESS_CLANG_2( x )  ".ARM.__at_" #x
        #define RETARGET_BUILD_INFO_ADDRESS_CLANG_1( x )   RETARGET_BUILD_INFO_ADDRESS_CLANG_2( x )
        #define RETARGET_BUILD_INFO_ADDRESS_CLANG          RETARGET_BUILD_INFO_ADDRESS_CLANG_1( RETARGET_BUILD_INFO_ADDRESS )

        // опциональное размещение по фиксированному адресу
        #ifdef RETARGET_BUILD_INFO_ADDRESS

            // keil armcc
            #if defined ( __CC_ARM )

                #define RETARGET_BUILD_INFO_ADDRESS_ATTRIB __attribute__((at(RETARGET_BUILD_INFO_ADDRESS)))

            // keil armclang
            #elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)

                #define RETARGET_BUILD_INFO_ADDRESS_ATTRIB __attribute__((section(RETARGET_BUILD_INFO_ADDRESS_CLANG)))
                
            // gcc
            #elif defined ( __GNUC__ )
            
                #define RETARGET_BUILD_INFO_ADDRESS_ATTRIB __attribute__((at(RETARGET_BUILD_INFO_ADDRESS))) 
            #else                
                #error "Your compiler is not supported yet."
            #endif

        // размещение где-нибудь
        #else
        
            // keil armcc, armclang или gcc
            #if defined ( __CC_ARM ) || \
                ( defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) ) || \
                defined ( __GNUC__ )
                
                #define RETARGET_BUILD_INFO_ADDRESS_ATTRIB
                
            #else                
                #error "Your compiler is not supported yet."
            #endif

        #endif
            
        #define EMPTY_DATA  "no data"
        #define COMMIT_DATA "no data"

        const char date_time_commit[] __attribute__((used)) RETARGET_BUILD_INFO_ADDRESS_ATTRIB =
        {
            "compilation time: " __DATE__ ", " __TIME__ " commit_data: " COMMIT_DATA
        };
        
    // в эклипсе этот скрипт пока не вызывается, поэтому проверка необязательная
    #ifndef __GNUC__
    
        // Удостоверяемся, что информация о коммите не пустая - раз уж она включена
        static constexpr bool strings_equal(char const *one, char const *two)
        {
            return (*one && *two) ? (*one == *two && strings_equal(one + 1, two + 1)) : (!*one && !*two);
        }

        static_assert( strings_equal( COMMIT_DATA, EMPTY_DATA ) == false,
                        "Commit data should not be empty!\n"
                        "Please, check if update_commit_hash.bat is called correctly\n"
                        "If you don't want to use it, define UMBA_RETARGET_DISABLE_BUILD_INFO in your project");

    #endif

    }} // namespace

#endif

extern "C" 
{

    /**************************************************************************************************
    Описание:  Эта функция вызывается в конструкторе каждого объекта со статическим временем жизни,
               чтобы "зарегистрировать" в динамически выделяемом списке. После выхода из main, вызовется
               _sys_exit, который для всех этих объектов вызывает деструкторы. Поскольку у нас main 
               никогда не завершается, все это абсолютно бессмысленно и только зря память жрет.
               ARM официально разрешает переопределить эту функцию.
    Аргументы: Какие-то есть, но игнорируются.
    Возврат:   Положительное число означает, что все хорошо.
    Замечания: Гуглите __aeabi_atexit и читайте официальную доку от ARM, если хотите узнать больше.
               Актуально только для ARMCC
    **************************************************************************************************/
    int __aeabi_atexit(void)
    {
        return 1;
    }
    // эти atexit'ы тоже позволяют регистрировать функции, которые вызовуться после выхода из main
    // они тоже бессмысленные
    // ненулевой возврат означает, что регистрация не удалась - и пес с ней
    // их переопределение позволяет сэкономить несколько десятков байт
    int __cxa_atexit(void)
    {
        return 1;
    }
    
    int atexit(void)
    {
        return 1;
    }
   
    // вызывается после main, тоже бесполезно
    void exit(int return_code) 
    {
        while(1);
    }

    /**************************************************************************************************
    Описание:  Эта функция вызывается, если ассерт в периферийной библиотеке не смог.
    Аргументы: file - имя файла, в котором ассерт сработал, line - номер строки
    Возврат:   -
    Замечания: Вызывает обычный UMBA_ASSERT и повисает.
    **************************************************************************************************/

    // классический ассерт для STM32
    #ifdef USE_FULL_ASSERT
        void assert_failed(uint8_t * file, uint32_t line)
        { 
            /* User can add his own implementation to report the file name and line number,
             ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
             
            (void)file;
            (void)line;

            UMBA_ASSERT_FAIL();
        }
    #endif

    // варианты для миландра 
    #if (USE_ASSERT_INFO == 1)    
        void assert_failed(uint32_t file_id, uint32_t line)
        {
            (void)file_id;
            (void)line;
        
            UMBA_ASSERT_FAIL();
        }
    #elif (USE_ASSERT_INFO == 2)

        void assert_failed(uint32_t file_id, uint32_t line, const uint8_t * expr)
        {
            (void)file_id;
            (void)line;
            (void)expr;
        
            UMBA_ASSERT_FAIL();
        }
    #endif 

}

/***************************************************************************************************
  Следующий блок функций делает возможным использования printf в симуляторе keil'a.
  Чтобы выключить - объявите символ UMBA_DONT_USE_RETARGET.
  
  Данная реализация является минимальной и не будет работать вне симулятора.
  
***************************************************************************************************/
#ifndef UMBA_DONT_USE_RETARGET 

    // если компилятор - armcc или keil-clang
    #if __CC_ARM || ( (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) )

        #if ( (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) )

            asm(".global __use_no_semihosting_swi\n");
            
        #elif __CC_ARM
        
            #pragma import(__use_no_semihosting_swi)
            
            namespace std { struct __FILE { int handle;} ; }
        
        #endif
        
        #include <stdio.h>
        #include <rt_sys.h>
        #include <rt_misc.h>


        std::FILE std::__stdout;
        std::FILE std::__stdin;
        std::FILE std::__stderr;
        
        extern "C"
        { 
            int fputc(int c, FILE *f)
            {
                return ITM_SendChar(c);
            }

            int fgetc(FILE *f)
            {
                char ch = 0;

                return((int)ch);
            }

            int ferror(FILE *f)
            {
                /* Your implementation of ferror */
                return EOF;
            }

            void _ttywrch(int ch)
            {
                ITM_SendChar(ch);            
            }
            
            char *_sys_command_string(char *cmd, int len)
            {
                return NULL;
            }
            
            // вызывается после main
            void _sys_exit(int return_code) 
            {
                while(1);
            }        
        }
    #endif

#endif

/***************************************************************************************************
  Этот кусок нужен только для armclang  
***************************************************************************************************/

#if ( (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) )

    // это чтобы на оптимизации -О0 armclang не пытался парсить аргументы для main'a
    __asm(".global __ARM_use_no_argv");

#endif
