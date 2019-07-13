#include <windows.h>
#include <stdlib.h>
#include <fstream>
LPSTR ClassName = "WNDClass";
MSG message;
int frame = 20;
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	std::fstream F;
	F.open("../../MAPOWANIE/example/OGmap.txt", std::ios::in);
	if (F.good() == 0)
	{
		MessageBox(NULL, "Map data file doesn't exist", "ERROR", MB_ICONEXCLAMATION | MB_OK);
		return 1;
	}
	int i, j, N;
	F >> N;
	double **OGmap = new double *[N];
	for (i = 0; i < N; i++) OGmap[i] = new double[N];
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++) F >> OGmap[i][j];
	} 
	WNDCLASSEX wc;
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = 0;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 2);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = ClassName;
	wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	if (!RegisterClassEx(&wc))
	{
		MessageBox(NULL, "Registration failed", "ERROR", MB_ICONEXCLAMATION | MB_OK);
		return 1;
	}
	HWND hwnd;
	hwnd = CreateWindowEx(WS_EX_CLIENTEDGE, ClassName, "Occupancy grid map", WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, N+3*frame, N+40+2*frame, NULL, NULL, hInstance, NULL);
	if (hwnd == NULL)
	{
		MessageBox(NULL, "Fail to create window", "ERROR", MB_ICONEXCLAMATION);
		return 1;
	}
	ShowWindow(hwnd, nCmdShow);
	HDC hdc = GetDC(hwnd);
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			if(OGmap[i][j]<0.2) SetPixel(hdc, j+frame, i+frame, RGB(255, 255, 255));
			if(OGmap[i][j]>0.8) SetPixel(hdc, j+frame, i+frame, RGB(0, 0, 0));
			if(OGmap[i][j] <= 0.8&&OGmap[i][j] >= 0.2) SetPixel(hdc, j+frame, i+frame, RGB(160, 160, 160));
		}
	}
	UpdateWindow(hwnd);
	ReleaseDC(hwnd, hdc);
	while (GetMessage(&message, NULL, 0, 0))
	{
		TranslateMessage(&message);
		DispatchMessage(&message);
	}
	return message.wParam;
	F.close();
	ReleaseDC(hwnd, hdc);
}
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_CLOSE:
		DestroyWindow(hwnd);
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	default:
		return DefWindowProc(hwnd, msg, wParam, lParam);
	}

	return 0;
}