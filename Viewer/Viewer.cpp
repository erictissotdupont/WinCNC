//--------------------------------------------------------------------------------------
// File: Viewer.cpp
//
// This application demonstrates simple lighting in the vertex shader
//
// http://msdn.microsoft.com/en-us/library/windows/apps/ff729723.aspx
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#include <windows.h>
#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include <directxcolors.h>
#include "resource.h"

using namespace DirectX;

//--------------------------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------------------------
struct SimpleVertex
{
    XMFLOAT3 Pos;
    XMFLOAT3 Normal;
};


struct ConstantBuffer
{
	XMMATRIX mWorld;
	XMMATRIX mView;
	XMMATRIX mProjection;
	XMFLOAT4 vLightDir[2];
	XMFLOAT4 vLightColor[2];
	XMFLOAT4 vOutputColor;
};


typedef struct
{
	DWORD id;
	DWORD dx;
	DWORD dy;
	float res;
	DWORD cbAlt;
} header_t;


//--------------------------------------------------------------------------------------
// Global Variables
//--------------------------------------------------------------------------------------
HINSTANCE               g_hInst = nullptr;
HWND                    g_hWnd = nullptr;
D3D_DRIVER_TYPE         g_driverType = D3D_DRIVER_TYPE_NULL;
D3D_FEATURE_LEVEL       g_featureLevel = D3D_FEATURE_LEVEL_11_0;
ID3D11Device*           g_pd3dDevice = nullptr;
ID3D11Device1*          g_pd3dDevice1 = nullptr;
ID3D11DeviceContext*    g_pImmediateContext = nullptr;
ID3D11DeviceContext1*   g_pImmediateContext1 = nullptr;
IDXGISwapChain*         g_pSwapChain = nullptr;
IDXGISwapChain1*        g_pSwapChain1 = nullptr;
ID3D11RenderTargetView* g_pRenderTargetView = nullptr;
ID3D11Texture2D*        g_pDepthStencil = nullptr;
ID3D11DepthStencilView* g_pDepthStencilView = nullptr;
ID3D11VertexShader*     g_pVertexShader = nullptr;
ID3D11PixelShader*      g_pPixelShader = nullptr;
ID3D11PixelShader*      g_pPixelShaderSolid = nullptr;
ID3D11InputLayout*      g_pVertexLayout = nullptr;
ID3D11Buffer*           g_pVertexBuffer = nullptr;
ID3D11Buffer*           g_pIndexBuffer = nullptr;
ID3D11Buffer*           g_pConstantBuffer = nullptr;
XMMATRIX                g_World;
XMMATRIX                g_View;
XMMATRIX                g_Projection;

float					g_tZ = 0.0f;
float					g_tY = 0.0f;

DWORD					g_vCount = 0;
SimpleVertex*			g_vertices = NULL;

DWORD					g_iCount = 0;
DWORD*					indices = NULL;

DWORD					g_index = 0;
WCHAR					g_szAltDataFile[MAX_PATH];
HANDLE					g_hRender = NULL;
HANDLE					g_hMapFile = NULL;
HANDLE					g_hFileChangeEvent = NULL;

float*					g_alt = NULL;
header_t				g_header;

//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------
HRESULT InitWindow( HINSTANCE hInstance, int nCmdShow );
HRESULT InitDevice();
void CleanupDevice();
LRESULT CALLBACK    WndProc( HWND, UINT, WPARAM, LPARAM );
void Render();
HRESULT LoadAltitudeFile();


DWORD FileChangeWatcherThread()
{
	while (1)
	{
		if (WaitForSingleObject(g_hFileChangeEvent, INFINITE) == WAIT_OBJECT_0)
		{
			SendMessage(g_hWnd, WM_USER, 1, 0);
			Sleep(1);
		}
		else
		{
			// Abort
			break;
		}
	}
	return 0;
}

//--------------------------------------------------------------------------------------
// Entry point to the program. Initializes everything and goes into a message processing 
// loop. Idle time is used to render the scene.
//--------------------------------------------------------------------------------------
int WINAPI wWinMain( _In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow )
{
    UNREFERENCED_PARAMETER( hPrevInstance );
    
	wcscpy_s(g_szAltDataFile, lpCmdLine);

    if( FAILED( InitWindow( hInstance, nCmdShow ) ) )
        return 0;

    if( FAILED( InitDevice() ) )
    {
        CleanupDevice();
        return 0;
    }

    // Main message loop
    MSG msg = {0};
    while( WM_QUIT != msg.message )
    {
        if( GetMessage( &msg, nullptr, 0, 0 ) )
        {
            TranslateMessage( &msg );
            DispatchMessage( &msg );
        }
    }

    CleanupDevice();

    return ( int )msg.wParam;
}


//--------------------------------------------------------------------------------------
// Register class and create window
//--------------------------------------------------------------------------------------
HRESULT InitWindow( HINSTANCE hInstance, int nCmdShow )
{
    // Register class
    WNDCLASSEX wcex;
    wcex.cbSize = sizeof( WNDCLASSEX );
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon( hInstance, ( LPCTSTR )IDI_TUTORIAL1 );
    wcex.hCursor = LoadCursor( nullptr, IDC_ARROW );
    wcex.hbrBackground = ( HBRUSH )( COLOR_WINDOW + 1 );
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = L"TutorialWindowClass";
    wcex.hIconSm = LoadIcon( wcex.hInstance, ( LPCTSTR )IDI_TUTORIAL1 );
    if( !RegisterClassEx( &wcex ) )
        return E_FAIL;

    // Create window
    g_hInst = hInstance;
    RECT rc = { 0, 0, 1900, 1200 };
    AdjustWindowRect( &rc, WS_OVERLAPPEDWINDOW, FALSE );
    g_hWnd = CreateWindow( L"TutorialWindowClass", L"Direct3D 11 Tutorial 6", WS_OVERLAPPEDWINDOW,
                           CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hInstance,
                           nullptr );
    if( !g_hWnd )
        return E_FAIL;

    ShowWindow( g_hWnd, nCmdShow );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Helper for compiling shaders with D3DCompile
//
// With VS 11, we could load up prebuilt .cso files instead...
//--------------------------------------------------------------------------------------
HRESULT CompileShaderFromFile( WCHAR* szFileName, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut )
{
    HRESULT hr = S_OK;

    DWORD dwShaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
    // Set the D3DCOMPILE_DEBUG flag to embed debug information in the shaders.
    // Setting this flag improves the shader debugging experience, but still allows 
    // the shaders to be optimized and to run exactly the way they will run in 
    // the release configuration of this program.
    dwShaderFlags |= D3DCOMPILE_DEBUG;

    // Disable optimizations to further improve shader debugging
    dwShaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

    ID3DBlob* pErrorBlob = nullptr;
    hr = D3DCompileFromFile( szFileName, nullptr, nullptr, szEntryPoint, szShaderModel, 
        dwShaderFlags, 0, ppBlobOut, &pErrorBlob );
    if( FAILED(hr) )
    {
        if( pErrorBlob )
        {
            OutputDebugStringA( reinterpret_cast<const char*>( pErrorBlob->GetBufferPointer() ) );
            pErrorBlob->Release();
        }
        return hr;
    }
    if( pErrorBlob ) pErrorBlob->Release();

    return S_OK;
}


BOOL UpdateAltitude( )
{
	int n = 0;
	DWORD iX, iY;

	n = 0;
	for (iX = 0; iX<g_header.dx; iX++) for (iY = 0; iY<g_header.dy; iY++)
	{
		g_vertices[n].Pos.y = g_alt[n]; // g_alt[iX*g_header.dy + iY];
		n++;
	}

	n = 0;
	BOOL bUpdateScreen = FALSE;
	BOOL bUpdateVertex;

	for (iX = 0; iX < g_header.dx - 1; iX++) for (iY = 0; iY < g_header.dy - 1; iY++)
	{
		int a = g_header.dy * iX + iY;
		int b = a + 1;
		int c = a + g_header.dy;
		int d = c + 1;
		float l;
		XMFLOAT3 V1, V2, P;

		/*
		bUpdateVertex = FALSE;
		if (g_vertices[a].Pos.y != g_alt[a]) { g_vertices[a].Pos.y = g_alt[a]; bUpdateVertex = TRUE; }
		if (g_vertices[b].Pos.y != g_alt[b]) { g_vertices[b].Pos.y = g_alt[b]; bUpdateVertex = TRUE; }
		if (g_vertices[c].Pos.y != g_alt[c]) { g_vertices[c].Pos.y = g_alt[c]; bUpdateVertex = TRUE; }
		if (g_vertices[d].Pos.y != g_alt[d]) { g_vertices[d].Pos.y = g_alt[d]; bUpdateVertex = TRUE; }
		if (!bUpdateVertex) continue;
		bUpdateScreen = TRUE;
		*/

#define CP(u,v)		u.y*v.z - u.z*v.y, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x
#define DP(u,v)		(u.x*v.x + u.y*v.y + u.z*v.z)
#define LENGTH(u)	((float)(sqrt(DP(u,u))))
#define V(i,j)		g_vertices[i].Pos.x - g_vertices[j].Pos.x, g_vertices[i].Pos.y - g_vertices[j].Pos.y, g_vertices[i].Pos.z - g_vertices[j].Pos.z

		if (((iX + iY) & 1) == 0)
		{
			V1 = XMFLOAT3(V(a, d));
			V2 = XMFLOAT3(V(a, c));
		}
		else
		{
			V1 = XMFLOAT3(V(a, b));
			V2 = XMFLOAT3(V(a, c));
		}
		P = XMFLOAT3(CP(V1, V2));
		l = LENGTH(P);
		P.x = P.x / l;
		P.y = P.y / l;
		P.z = P.z / l;
		g_vertices[a].Normal = P;
		if (iX == g_header.dx - 1) g_vertices[c].Normal = P;
		if (iY == g_header.dy - 1) g_vertices[b].Normal = P;
	}

	return bUpdateScreen;
}

HRESULT LoadAltitudeFile( )
{
	static HANDLE hFile = INVALID_HANDLE_VALUE;
	static DWORD cbAlt = 0;
	DWORD iX, iY;
	int n = 0;

	hFile = CreateFile(g_szAltDataFile, FILE_READ_ACCESS, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
	{
		return S_FALSE;
	}
	
	ReadFile(hFile, &g_header, sizeof(header_t), NULL, NULL);

	cbAlt = g_header.dx * g_header.dy * sizeof(float);

	if (g_alt) free(g_alt);
	g_alt = (float*)malloc(cbAlt);

	ReadFile(hFile, g_alt, cbAlt, NULL, NULL);
	CloseHandle(hFile);

	int iXsz = g_header.dx;			// 1 + Xsz / resolution;
	int iYsz = g_header.dy;			// 1 + Ysz / resolution;

	// Number of g_vertices is same as points in file
	g_vCount = iXsz * iYsz;
	if (g_vertices == NULL)
	{
		g_vertices = (SimpleVertex*)malloc(g_vCount * sizeof(SimpleVertex));

		// Number of squares, 2 triangles per squares, 3 points per triangle
		g_iCount = (iXsz - 1) * (iYsz - 1) * 2 * 3;
		indices = (DWORD*)malloc(g_iCount * sizeof(DWORD));

		n = 0;
		for (iX = 0; iX < g_header.dx - 1; iX++) for (iY = 0; iY < g_header.dy - 1; iY++)
		{
			int a = g_header.dy * iX + iY;
			int b = a + 1;
			int c = a + g_header.dy;
			int d = c + 1;

			if (((iX + iY) & 1) == 0)
			{
				indices[n++] = a;
				indices[n++] = b;
				indices[n++] = d;

				indices[n++] = a;
				indices[n++] = d;
				indices[n++] = c;
			}
			else
			{
				indices[n++] = a;
				indices[n++] = b;
				indices[n++] = c;

				indices[n++] = b;
				indices[n++] = d;
				indices[n++] = c;
			}
		}

		n = 0;
		float Xsz = (g_header.dx - 1) * g_header.res; // 6.0f;
		float Ysz = (g_header.dy - 1) * g_header.res; // 5.0f;
		float Xoffset = -(Xsz / 2.0f);
		float Yoffset = -(Ysz / 2.0f);
		float x, z;

		for (iX = 0; iX < g_header.dx; iX++) for (iY = 0; iY < g_header.dy; iY++)
		{
			x = Xoffset + iX * g_header.res;
			z = Yoffset + iY * g_header.res;
			g_vertices[n].Pos = XMFLOAT3(x, 0, z);
			n++;
		}
	}

	UpdateAltitude( );

	return S_OK;
}


/*
HRESULT LoadAltitudeFile()
{
	static HANDLE hFile = INVALID_HANDLE_VALUE;
	static DWORD cbAlt;
	static float* alt = NULL;
	static header_t h;
	DWORD iX, iY;
	int n = 0;
	
	if (hFile == INVALID_HANDLE_VALUE)
	{
		hFile = CreateFile(g_szAltDataFile, FILE_READ_ACCESS, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
		if (hFile == INVALID_HANDLE_VALUE)
		{
			return S_FALSE;
		}
		ReadFile(hFile, &h, sizeof(h), NULL, NULL);
		cbAlt = h.dx * h.dy * sizeof(float);
		alt = (float*)malloc(cbAlt);

		int iXsz = h.dx;			// 1 + Xsz / resolution;
		int iYsz = h.dy;			// 1 + Ysz / resolution;

		// Number of g_vertices is same as points in file
		g_vCount = iXsz * iYsz;
		g_vertices = (SimpleVertex*)malloc(g_vCount * sizeof(SimpleVertex));

		// Number of squares, 2 triangles per squares, 3 points per triangle
		g_iCount = (iXsz - 1) * (iYsz - 1) * 2 * 3;       
		indices = (DWORD*)malloc(g_iCount * sizeof(DWORD));

		n = 0;
		for (iX = 0; iX < h.dx - 1; iX++) for (iY = 0; iY < h.dy - 1; iY++)
		{
			int a = h.dy * iX + iY;
			int b = a + 1;
			int c = a + h.dy;
			int d = c + 1;

			if (((iX + iY) & 1) == 0)
			{
				indices[n++] = a;
				indices[n++] = b;
				indices[n++] = d;

				indices[n++] = a;
				indices[n++] = d;
				indices[n++] = c;
			}
			else
			{
				indices[n++] = a;
				indices[n++] = b;
				indices[n++] = c;

				indices[n++] = b;
				indices[n++] = d;
				indices[n++] = c;
			}
		}

		n = 0;
		float Xsz = (h.dx - 1) * h.res; // 6.0f;
		float Ysz = (h.dy - 1) * h.res; // 5.0f;
		float Xoffset = -(Xsz / 2.0f);
		float Yoffset = -(Ysz / 2.0f);
		float x,z;

		for (iX = 0; iX<h.dx; iX++) for (iY = 0; iY<h.dy; iY++)
		{
			x = Xoffset + iX * h.res;
			z = Yoffset + iY * h.res;
			g_vertices[n].Pos = XMFLOAT3(x, 0, z);
			n++;
		}
	}
	else
	{
		SetFilePointer(hFile, sizeof(h), NULL, FILE_BEGIN);
	}
	
	ReadFile(hFile, alt, cbAlt, NULL, NULL);

	n = 0;
	for (iX = 0; iX<h.dx; iX++) for (iY = 0; iY<h.dy; iY++)
	{
		g_vertices[n++].Pos.y = alt[iX*h.dy + iY];
	}

	n = 0;
	for (iX = 0; iX < h.dx - 1; iX++) for (iY = 0; iY < h.dy - 1; iY++)
	{
		int a = h.dy * iX + iY;
		int b = a + 1;
		int c = a + h.dy;
		int d = c + 1;
		float l;
		XMFLOAT3 V1, V2, P;

#define CP(u,v)		u.y*v.z - u.z*v.y, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x
#define DP(u,v)		(u.x*v.x + u.y*v.y + u.z*v.z)
#define LENGTH(u)	sqrt(DP(u,u))
#define V(i,j)		g_vertices[i].Pos.x - g_vertices[j].Pos.x, g_vertices[i].Pos.y - g_vertices[j].Pos.y, g_vertices[i].Pos.z - g_vertices[j].Pos.z

		if (((iX + iY) & 1) == 0)
		{
			V1 = XMFLOAT3(V(a, d));
			V2 = XMFLOAT3(V(a, c));
		}
		else
		{
			V1 = XMFLOAT3(V(a, b));
			V2 = XMFLOAT3(V(a, c));
		}
		P = XMFLOAT3(CP(V1, V2));
		l = LENGTH(P);
		P.x = P.x / l;
		P.y = P.y / l;
		P.z = P.z / l;
		g_vertices[a].Normal = P;
		if (iX == h.dx - 1) g_vertices[c].Normal = P;
		if (iY == h.dy - 1) g_vertices[b].Normal = P;
	}

	//CloseHandle(hFile);

	return S_OK;
}
*/


DWORD UpdateVerticesThread(LPVOID pParam)
{
	// SimpleVertex *g_vertices = (SimpleVertex*)pParam;

	while (1)
	{
		Sleep(250);
		
		UpdateAltitude();
		
		D3D11_MAPPED_SUBRESOURCE resource;
		WaitForSingleObject(g_hRender, INFINITE);
		g_pImmediateContext->Map(g_pVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
		memcpy(resource.pData, g_vertices, sizeof(SimpleVertex) * g_vCount);
		g_pImmediateContext->Unmap(g_pVertexBuffer, 0);
		ReleaseMutex(g_hRender);

		PostMessage(g_hWnd, WM_USER, 1, 0);
	}
	return 0;
}

//--------------------------------------------------------------------------------------
// Create Direct3D device and swap chain
//--------------------------------------------------------------------------------------
HRESULT InitDevice()
{
    HRESULT hr = S_OK;

    RECT rc;
    GetClientRect( g_hWnd, &rc );
    UINT width = rc.right - rc.left;
    UINT height = rc.bottom - rc.top;

    UINT createDeviceFlags = 0;
#ifdef _DEBUG
	// Removed to avoid having to install the D3D SDK for Windows 10
    // createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    D3D_DRIVER_TYPE driverTypes[] =
    {
        D3D_DRIVER_TYPE_HARDWARE,
        D3D_DRIVER_TYPE_WARP,
        D3D_DRIVER_TYPE_REFERENCE,
    };
    UINT numDriverTypes = ARRAYSIZE( driverTypes );

    D3D_FEATURE_LEVEL featureLevels[] =
    {
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1,
        D3D_FEATURE_LEVEL_10_0,
    };
	UINT numFeatureLevels = ARRAYSIZE( featureLevels );

    for( UINT driverTypeIndex = 0; driverTypeIndex < numDriverTypes; driverTypeIndex++ )
    {
        g_driverType = driverTypes[driverTypeIndex];
        hr = D3D11CreateDevice( nullptr, g_driverType, nullptr, createDeviceFlags, featureLevels, numFeatureLevels,
                                D3D11_SDK_VERSION, &g_pd3dDevice, &g_featureLevel, &g_pImmediateContext );

        if ( hr == E_INVALIDARG )
        {
            // DirectX 11.0 platforms will not recognize D3D_FEATURE_LEVEL_11_1 so we need to retry without it
            hr = D3D11CreateDevice( nullptr, g_driverType, nullptr, createDeviceFlags, &featureLevels[1], numFeatureLevels - 1,
                                    D3D11_SDK_VERSION, &g_pd3dDevice, &g_featureLevel, &g_pImmediateContext );
        }

        if( SUCCEEDED( hr ) )
            break;
    }
    if( FAILED( hr ) )
        return hr;

    // Obtain DXGI factory from device (since we used nullptr for pAdapter above)
    IDXGIFactory1* dxgiFactory = nullptr;
    {
        IDXGIDevice* dxgiDevice = nullptr;
        hr = g_pd3dDevice->QueryInterface( __uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgiDevice) );
        if (SUCCEEDED(hr))
        {
            IDXGIAdapter* adapter = nullptr;
            hr = dxgiDevice->GetAdapter(&adapter);
            if (SUCCEEDED(hr))
            {
                hr = adapter->GetParent( __uuidof(IDXGIFactory1), reinterpret_cast<void**>(&dxgiFactory) );
                adapter->Release();
            }
            dxgiDevice->Release();
        }
    }
    if (FAILED(hr))
        return hr;

    // Create swap chain
    IDXGIFactory2* dxgiFactory2 = nullptr;
    hr = dxgiFactory->QueryInterface( __uuidof(IDXGIFactory2), reinterpret_cast<void**>(&dxgiFactory2) );
    if ( dxgiFactory2 )
    {
        // DirectX 11.1 or later
        hr = g_pd3dDevice->QueryInterface( __uuidof(ID3D11Device1), reinterpret_cast<void**>(&g_pd3dDevice1) );
        if (SUCCEEDED(hr))
        {
            (void) g_pImmediateContext->QueryInterface( __uuidof(ID3D11DeviceContext1), reinterpret_cast<void**>(&g_pImmediateContext1) );
        }

        DXGI_SWAP_CHAIN_DESC1 sd;
        ZeroMemory(&sd, sizeof(sd));
        sd.Width = width;
        sd.Height = height;
        sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.SampleDesc.Count = 1;
        sd.SampleDesc.Quality = 0;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.BufferCount = 1;

        hr = dxgiFactory2->CreateSwapChainForHwnd( g_pd3dDevice, g_hWnd, &sd, nullptr, nullptr, &g_pSwapChain1 );
        if (SUCCEEDED(hr))
        {
            hr = g_pSwapChain1->QueryInterface( __uuidof(IDXGISwapChain), reinterpret_cast<void**>(&g_pSwapChain) );
        }

        dxgiFactory2->Release();
    }
    else
    {
        // DirectX 11.0 systems
        DXGI_SWAP_CHAIN_DESC sd;
        ZeroMemory(&sd, sizeof(sd));
        sd.BufferCount = 1;
        sd.BufferDesc.Width = width;
        sd.BufferDesc.Height = height;
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.BufferDesc.RefreshRate.Numerator = 60;
        sd.BufferDesc.RefreshRate.Denominator = 1;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.OutputWindow = g_hWnd;
        sd.SampleDesc.Count = 1;
        sd.SampleDesc.Quality = 0;
        sd.Windowed = TRUE;

        hr = dxgiFactory->CreateSwapChain( g_pd3dDevice, &sd, &g_pSwapChain );
    }

    dxgiFactory->Release();

    if (FAILED(hr))
        return hr;

    // Create a render target view
    ID3D11Texture2D* pBackBuffer = nullptr;
    hr = g_pSwapChain->GetBuffer( 0, __uuidof( ID3D11Texture2D ), reinterpret_cast<void**>( &pBackBuffer ) );
    if( FAILED( hr ) )
        return hr;

    hr = g_pd3dDevice->CreateRenderTargetView( pBackBuffer, nullptr, &g_pRenderTargetView );
    pBackBuffer->Release();
    if( FAILED( hr ) )
        return hr;

    // Create depth stencil texture
    D3D11_TEXTURE2D_DESC descDepth;
	ZeroMemory( &descDepth, sizeof(descDepth) );
    descDepth.Width = width;
    descDepth.Height = height;
    descDepth.MipLevels = 1;
    descDepth.ArraySize = 1;
    descDepth.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    descDepth.SampleDesc.Count = 1;
    descDepth.SampleDesc.Quality = 0;
    descDepth.Usage = D3D11_USAGE_DEFAULT;
    descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    descDepth.CPUAccessFlags = 0;
    descDepth.MiscFlags = 0;
    hr = g_pd3dDevice->CreateTexture2D( &descDepth, nullptr, &g_pDepthStencil );
    if( FAILED( hr ) )
        return hr;

    // Create the depth stencil view
    D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
	ZeroMemory( &descDSV, sizeof(descDSV) );
    descDSV.Format = descDepth.Format;
    descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
    descDSV.Texture2D.MipSlice = 0;
    hr = g_pd3dDevice->CreateDepthStencilView( g_pDepthStencil, &descDSV, &g_pDepthStencilView );
    if( FAILED( hr ) )
        return hr;

    g_pImmediateContext->OMSetRenderTargets( 1, &g_pRenderTargetView, g_pDepthStencilView );

    // Setup the viewport
    D3D11_VIEWPORT vp;
    vp.Width = (FLOAT)width;
    vp.Height = (FLOAT)height;
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    vp.TopLeftX = 0;
    vp.TopLeftY = 0;
    g_pImmediateContext->RSSetViewports( 1, &vp );

	// Compile the vertex shader
	ID3DBlob* pVSBlob = nullptr;
    hr = CompileShaderFromFile( L"Viewer.fx", "VS", "vs_4_0", &pVSBlob );
    if( FAILED( hr ) )
    {
        MessageBox( nullptr,
                    L"The FX file cannot be compiled.  Please run this executable from the directory that contains the FX file.", L"Error", MB_OK );
        return hr;
    }

	// Create the vertex shader
	hr = g_pd3dDevice->CreateVertexShader( pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &g_pVertexShader );
	if( FAILED( hr ) )
	{	
		pVSBlob->Release();
        return hr;
	}

    // Define the input layout
    D3D11_INPUT_ELEMENT_DESC layout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	UINT numElements = ARRAYSIZE( layout );

    // Create the input layout
	hr = g_pd3dDevice->CreateInputLayout( layout, numElements, pVSBlob->GetBufferPointer(),
                                          pVSBlob->GetBufferSize(), &g_pVertexLayout );
	pVSBlob->Release();
	if( FAILED( hr ) )
        return hr;

    // Set the input layout
    g_pImmediateContext->IASetInputLayout( g_pVertexLayout );

	// Compile the pixel shader
	ID3DBlob* pPSBlob = nullptr;
    hr = CompileShaderFromFile( L"Viewer.fx", "PS", "ps_4_0", &pPSBlob );
    if( FAILED( hr ) )
    {
        MessageBox( nullptr,
                    L"The FX file cannot be compiled.  Please run this executable from the directory that contains the FX file.", L"Error", MB_OK );
        return hr;
    }

	// Create the pixel shader
	hr = g_pd3dDevice->CreatePixelShader( pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &g_pPixelShader );
	pPSBlob->Release();
    if( FAILED( hr ) )
        return hr;

	// Compile the pixel shader
	pPSBlob = nullptr;
	hr = CompileShaderFromFile( L"Viewer.fx", "PSSolid", "ps_4_0", &pPSBlob );
    if( FAILED( hr ) )
    {
        MessageBox( nullptr,
                    L"The FX file cannot be compiled.  Please run this executable from the directory that contains the FX file.", L"Error", MB_OK );
        return hr;
    }

	// Create the pixel shader
	hr = g_pd3dDevice->CreatePixelShader( pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &g_pPixelShaderSolid );
	pPSBlob->Release();
    if( FAILED( hr ) )
        return hr;

	hr = LoadAltitudeFile();
	if (FAILED(hr))
		return hr;

	D3D11_BUFFER_DESC bd;
	ZeroMemory(&bd, sizeof(bd));
	bd.Usage = D3D11_USAGE_DYNAMIC; // D3D11_USAGE_DEFAULT
	bd.ByteWidth = sizeof(SimpleVertex) * g_vCount;
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	
	D3D11_SUBRESOURCE_DATA InitData;
	ZeroMemory(&InitData, sizeof(InitData));
	InitData.pSysMem = g_vertices;
	hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pVertexBuffer);
	if (FAILED(hr))
		return hr;

	// Set vertex buffer
	UINT stride = sizeof(SimpleVertex);
	UINT offset = 0;
	g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pVertexBuffer, &stride, &offset);

	bd.Usage = D3D11_USAGE_DEFAULT; 
	bd.ByteWidth = sizeof(DWORD) * g_iCount;
	bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	bd.CPUAccessFlags = 0;
	InitData.pSysMem = indices;
	hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pIndexBuffer);
	if (FAILED(hr))
		return hr;

    // Set index buffer
    g_pImmediateContext->IASetIndexBuffer( g_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0 );

    // Set primitive topology
    g_pImmediateContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );

	// Create the constant buffer
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.ByteWidth = sizeof(ConstantBuffer);
	bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	bd.CPUAccessFlags = 0;
    hr = g_pd3dDevice->CreateBuffer( &bd, nullptr, &g_pConstantBuffer );
    if( FAILED( hr ))
        return hr;

    // Initialize the world matrices
	g_World = XMMatrixIdentity();

    // Initialize the view matrix
	XMVECTOR Eye = XMVectorSet( 0.0f, 5.0f, -5.0f, 0.0f );
	XMVECTOR At = XMVectorSet( 0.0f, 0.0f, 0.0f, 0.0f );
	XMVECTOR Up = XMVectorSet( 0.0f, 1.0f, 0.0f, 0.0f );
	g_View = XMMatrixLookAtLH( Eye, At, Up );

    // Initialize the projection matrix
	g_Projection = XMMatrixPerspectiveFovLH( XM_PIDIV4, width / (FLOAT)height, 0.01f, 100.0f );

	g_hRender = CreateMutex(NULL, FALSE, NULL);

	Render();

	g_hFileChangeEvent = CreateEvent(NULL, FALSE, FALSE, L"Local\\AltFileChangeEvent");
	// FindFirstChangeNotification(g_szAltDataFile, FALSE, FILE_NOTIFY_CHANGE_LAST_WRITE | FILE_NOTIFY_CHANGE_LAST_ACCESS );
	if (g_hFileChangeEvent)
	{
		DWORD dwThreadId;
		CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)FileChangeWatcherThread, NULL, 0, &dwThreadId);
	}
	
    return S_OK;
}


//--------------------------------------------------------------------------------------
// Render a frame
//--------------------------------------------------------------------------------------
void Render()
{
	WaitForSingleObject(g_hRender, INFINITE);

	// Rotate cube around the origin
	g_World = XMMatrixRotationY( g_tY );
	
	//g_View = XMMatrixRotationY(g_tY);

	float l1 = 0.2;
	float l2 = 0.5;

	// Setup our lighting parameters
	XMFLOAT4 vLightDirs[2] =
	{
		XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f),
		XMFLOAT4(1.0f, 0.2f, 1.0f, 1.0f),
	};
	XMFLOAT4 vLightColors[2] =
	{
		XMFLOAT4(l1, l1, l1, 1.0f),
		XMFLOAT4(l2, l2, l2, 1.0f)
	};

	// Rotate the second light around the origin
	XMMATRIX mRotate = XMMatrixRotationY(g_tY);
	XMVECTOR vLightDir = XMLoadFloat4(&vLightDirs[1]);
	vLightDir = XMVector3Transform(vLightDir, mRotate);
	XMStoreFloat4(&vLightDirs[1], vLightDir);

	//
	// Clear the back buffer
	//
	g_pImmediateContext->ClearRenderTargetView(g_pRenderTargetView, Colors::MidnightBlue);

	//
	// Clear the depth buffer to 1.0 (max depth)
	//
	g_pImmediateContext->ClearDepthStencilView(g_pDepthStencilView, D3D11_CLEAR_DEPTH, 1.0f, 0);

	//
	// Update matrix variables and lighting variables
	//
	ConstantBuffer cb1;
	cb1.mWorld = XMMatrixTranspose(g_World);
	cb1.mView = XMMatrixTranspose(g_View);
	cb1.mProjection = XMMatrixTranspose(g_Projection);
	cb1.vLightDir[0] = vLightDirs[0];
	cb1.vLightDir[1] = vLightDirs[1];
	cb1.vLightColor[0] = vLightColors[0];
	cb1.vLightColor[1] = vLightColors[1];
	cb1.vOutputColor = XMFLOAT4(0, 0, 0, 0);
	g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb1, 0, 0);

	//
	// Render the cube
	//
	g_pImmediateContext->VSSetShader(g_pVertexShader, nullptr, 0);
	g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
	g_pImmediateContext->PSSetShader(g_pPixelShader, nullptr, 0);
	g_pImmediateContext->PSSetConstantBuffers(0, 1, &g_pConstantBuffer);
	g_pImmediateContext->DrawIndexed(g_iCount, 0, 0);


	//
	// Render each light
	//
	/*
	for( int m = 0; m < 2; m++ )
	{
	XMMATRIX mLight = XMMatrixTranslationFromVector( 5.0f * XMLoadFloat4( &vLightDirs[m] ) );
	XMMATRIX mLightScale = XMMatrixScaling( 0.2f, 0.2f, 0.2f );
	mLight = mLightScale * mLight;

	// Update the world variable to reflect the current light
	cb1.mWorld = XMMatrixTranspose( mLight );
	cb1.vOutputColor = vLightColors[m];
	g_pImmediateContext->UpdateSubresource( g_pConstantBuffer, 0, nullptr, &cb1, 0, 0 );

	g_pImmediateContext->PSSetShader( g_pPixelShaderSolid, nullptr, 0 );
	g_pImmediateContext->DrawIndexed( g_iCount, 0, 0);
	}
	*/

	//
	// Present our back buffer to our front buffer
	//
	g_pSwapChain->Present(0, 0);

	ReleaseMutex(g_hRender);
}

//--------------------------------------------------------------------------------------
// Clean up the objects we've created
//--------------------------------------------------------------------------------------
void CleanupDevice()
{
    if( g_pImmediateContext ) g_pImmediateContext->ClearState();

    if( g_pConstantBuffer ) g_pConstantBuffer->Release();
    if( g_pVertexBuffer ) g_pVertexBuffer->Release();
    if( g_pIndexBuffer ) g_pIndexBuffer->Release();
    if( g_pVertexLayout ) g_pVertexLayout->Release();
    if( g_pVertexShader ) g_pVertexShader->Release();
    if( g_pPixelShaderSolid ) g_pPixelShaderSolid->Release();
    if( g_pPixelShader ) g_pPixelShader->Release();
    if( g_pDepthStencil ) g_pDepthStencil->Release();
    if( g_pDepthStencilView ) g_pDepthStencilView->Release();
    if( g_pRenderTargetView ) g_pRenderTargetView->Release();
    if( g_pSwapChain1 ) g_pSwapChain1->Release();
    if( g_pSwapChain ) g_pSwapChain->Release();
    if( g_pImmediateContext1 ) g_pImmediateContext1->Release();
    if( g_pImmediateContext ) g_pImmediateContext->Release();
    if( g_pd3dDevice1 ) g_pd3dDevice1->Release();
    if( g_pd3dDevice ) g_pd3dDevice->Release();
}

//--------------------------------------------------------------------------------------
// Called every time the application receives a message
//--------------------------------------------------------------------------------------
LRESULT CALLBACK WndProc( HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam )
{
    PAINTSTRUCT ps;
    HDC hdc;

    switch( message )
    {
		case WM_USER:
			if (wParam == 1)
			{
				D3D11_MAPPED_SUBRESOURCE resource;

				LoadAltitudeFile();

				g_pImmediateContext->Map(g_pVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
				memcpy(resource.pData, g_vertices, sizeof(SimpleVertex) * g_vCount);
				g_pImmediateContext->Unmap(g_pVertexBuffer, 0);

				Render();
			} 
			break;

		case WM_KEYDOWN:
			switch (wParam)
			{
			case VK_LEFT:
				g_tY += 0.1;
				Render();
				break;
			case VK_RIGHT:
				g_tY -= 0.1;
				Render();
				break;
			case VK_UP:
				g_index = g_index += 3;
				if (g_index >= g_iCount) g_index = 0;
				Render();
				break;
			case VK_DOWN:
				if (g_index == 0) g_index = g_iCount;
				g_index = g_index -= 3;
				Render();
				break;
			}
			break;

        case WM_PAINT:
            hdc = BeginPaint( hWnd, &ps );
            EndPaint( hWnd, &ps );
            break;

        case WM_DESTROY:
            PostQuitMessage( 0 );
            break;

        default:
            return DefWindowProc( hWnd, message, wParam, lParam );
    }

    return 0;
}
