/*****************************************************************************************
* Ce fichier contient les définitions des constantes qui contrôlent le programme principal
* (main.cpp): type d'évitements, type de programme, état initial du programme, etc.
*****************************************************************************************/

#define EVITEMENT_PASSIF
#define MATCH
#define DELAY_BEOFRE_GO 90

//Type de programme
#ifdef TEST_LINEAR 
    #define ENTRY_STATE IDLE
#endif
#ifdef TEST_ROTATE
    #define ENTRY_STATE IDLE
#endif
#ifdef MATCH
    #define ENTRY_STATE START  
#endif 
#ifdef TEST_STRAT
    #define ENTRY_STATE IDLE
#endif