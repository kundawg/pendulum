#include <unistd.h>
#include <ncurses.h>
#include "include/Controller.h"
#include "include/NatNetTypes.h"
#pragma warning( disable : 4996 )

int kbhit(void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

int getch_noblock() {
    if (kbhit())
        return getch();
    else
        return -1;
}

int overallMenuMode(Controller* controllerPointer)
{
    int c;
    int iResult = ErrorCode_OK;
    bool bExit = false;
    double k0,k1,k00,k01;
    double increaseFactor = 1.1;
    double decreaseFactor = 0.90909090909;
    bool gainsChanged = false;

//    initscr();
//    cbreak();
//    noecho();
//    scrollok(stdscr, TRUE);
//    nodelay(stdscr, TRUE);

    while (c = getch_noblock())
    {
        switch (c)
        {
            case 'y':
                controllerPointer->getGains(&k0, &k1);
                k0 = k0*increaseFactor;
                gainsChanged = true;
                break;
            case 'h':
                controllerPointer->getGains(&k0, &k1);
                k0 = k0*decreaseFactor;
                gainsChanged = true;
                break;
            case 'u':
                controllerPointer->getGains(&k0, &k1);
                k1 = k1*increaseFactor;
                gainsChanged = true;
                break;
            case 'j':
                controllerPointer->getGains(&k0, &k1);
                k1 = k1*decreaseFactor;
                gainsChanged = true;
                break;
            case 'i':
                controllerPointer->getGains(&k0, &k1);
                k00 = k00*increaseFactor;
                gainsChanged = true;
                break;
            case 'k':
                controllerPointer->getGains(&k0, &k1);
                k00 = k00*decreaseFactor;
                gainsChanged = true;
                break;
            case 'o':
                controllerPointer->getGains(&k0, &k1);
                k01 = k01*increaseFactor;
                gainsChanged = true;
                break;
            case 'l':
                controllerPointer->getGains(&k0, &k1);
                k01 = k01*decreaseFactor;
                gainsChanged = true;
                break;
            case 'd':
                controllerPointer->getDefaultGains(&k0, &k1);
                gainsChanged = true;
                break;
            case 'q':
                bExit = true;
                break;
            default:
                break;
        }
        if (bExit)
            break;
        if (gainsChanged)
        {
            controllerPointer->setGains(k0, k1);
            printf("new Gains: k0 = %4.3f, k1 = %4.3f", k0, k1);
            gainsChanged = false;
        }
        // ADDED A SLEEP HERE
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return iResult;
}

int main()
{
    std::cout << "Main Thread :: ID = " << std::this_thread::get_id() << std::endl;

    //start controller
    Controller controller;
    int errorCode = controller.initialize();
    if (errorCode != ErrorCode_OK)
    {
        printf("Something went wrong initializing the Controller.");
        return 1;
    }

    errorCode = overallMenuMode(&controller);
    if (errorCode != ErrorCode_OK)
    {
        printf("Something went wrong in menu mode.");
        return 1;
    }

    return ErrorCode_OK;
}