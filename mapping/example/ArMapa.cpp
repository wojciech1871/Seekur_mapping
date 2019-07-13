#include <iostream>
#include <fstream>
#include <cmath>
#include <fstream>

#define pi (4.0*atan(1.0))
using namespace std;

double fl;
void escape()
{
	fl = 1;
}
class ArMapa
{
	const double IXY;	//actual map size
	const int N;		//map resolution
	const double delta;	//size of one tile
	double **OGmap = new double *[N];
	double **logoddsmap = new double *[N];
	const double startAngle, endAngle, dfi;
	const int M;
	double *a = new double [M];
	double *b = new double [M];
	int *errcount = new int[M];
	int erroracc;	//liczba pomiarow "czysto", po jakich uzna ze jest ok
	fstream F;
	//fstream F2;
	double angletransf(double theta)
	{
		double a;
		a = atan2(sin(theta), cos(theta));
		return a;
	}
	double degtorad(double a)
	{
		double b;
		b = a*pi / 180;
		return b;
	}
	void transf(double x, double y, int &a, int &b) // retur indexes a, b of matrix for coordinates x, y
	{
		a = floor(x / delta);
		b = floor(y / delta);
	}
	void tilecenter(int a, int b, double &x, double &y)
	{
		x = a*delta + delta / 2;
		y = b*delta + delta / 2;
	}
	void inversebayes(int i, int j, bool p)
	{
		if (i < N && j < N && i >= 0 && j >= 0)
		{
			double pmxy;
			if (p == 1)
			{
				pmxy = 0.8;
				logoddsmap[i][j] = logoddsmap[i][j] + log(pmxy) - log(1 - pmxy);
			}
			if (p == 0)
			{
				pmxy = 0.2;
				logoddsmap[i][j] = logoddsmap[i][j] + log(pmxy) - log(1 - pmxy);
			}
		}
	}
	void BresenhamLine(int x1, int y1, int x2, int y2)	//algorithm for linear approximation
	{
		int d, dx, dy, ai, bi, xi, yi;
		int x = x1, y = y1;
		if (x1 < x2)
		{
			xi = 1;
			dx = x2 - x1;
		}
		else
		{
			xi = -1;
			dx = x1 - x2;
		}
		if (y1 < y2)
		{
			yi = 1;
			dy = y2 - y1;
		}
		else
		{
			yi = -1;
			dy = y1 - y2;
		}

		if (dx > dy)
		{
			ai = (dy - dx) * 2;
			bi = dy * 2;
			d = bi - dx;
			while (x != x2)
			{
				inversebayes(y, x, 0);
				if (d >= 0)
				{
					x += xi;
					y += yi;
					d += ai;
				}
				else
				{
					d += bi;
					x += xi;
				}
			}
		}
		else
		{
			ai = (dx - dy) * 2;
			bi = dx * 2;
			d = bi - dy;
			while (y != y2)
			{
				inversebayes(y, x, 0);
				if (d >= 0)
				{
					x += xi;
					y += yi;
					d += ai;
				}
				else
				{
					d += bi;
					y += yi;
				}
			}
		}
	}
	void myownAlgorithm(int x1, int y1, int x2, int y2, double rx, double ry, double theta, double b, double fi)
	{
		int dx, dy, xp, yp, xk, yk;
		double xcart, ycart;
		double beta, dist;
		double ddx, ddy;
		dx = x2 - x1;
		dy = y2 - y1;
		if (dx<0)
		{
			xp = x2 + 1;
			xk = x1 - 1;
		}
		if (dx > 0)
		{
			xp = x1 + 1;
			xk = x2 - 1;
		}
		if (dx == 0)
		{
			xp = x1;
			xk = x2;
		}
		if (dy < 0)
		{
			yp = y2 + 1;
			yk = y1 - 1;
		}
		if (dy>0)
		{
			yp = y1 + 1;
			yk = y2 - 1;
		}
		if (dy == 0)
		{
			yp = y1;
			yk = y2;
		}
		for (int x = xp; x <= xk; x++)
		{
			for (int y = yp; y <= yk; y++)
			{
				tilecenter(x, y, xcart, ycart);
				ddx = xcart - rx;
				ddy = ycart - ry;
				beta = atan2(ddy, ddx);
				if ((beta <= angletransf(theta + fi + degtorad(dfi)/2)) && (beta >= angletransf(theta + fi - degtorad(dfi)/2)))
				{
					dist = sqrt(ddx*ddx + ddy*ddy);
					if (dist <= b) inversebayes(y, x, 0);
				}
			}
		}
	}
	void measurement(ArRobot &robot)
	{
		double fi = startAngle;
		for (int i = 0; i < M; i++)
		{
			b[i] = robot.checkRangeDevicesCumulativePolar(fi - dfi / 2, fi + dfi / 2, &a[i]);
			if (b[i] > 4000)
			{
				a[i] = fi;
				if (errcount[i] != erroracc) errcount[i]++;
			}
			else errcount[i] = 0;
			//F2 << b[i] << "\t" << a[i] << "\t" << fi << endl;		//opcjonalny zapis do pliku
			fi += dfi;
		}
		//F2 << endl;
	}
public:
	ArMapa()
		: IXY(20000), N(500), delta(static_cast<double>(IXY) / static_cast<double>(N)), dfi(10), startAngle(-50), endAngle(50), M(static_cast<int>((endAngle - startAngle) / dfi) + 1),
		erroracc(12)
	{
		F.open("OGmap.txt", ios::out);
		//F2.open("pomiary.txt", ios::out);
		if (F.good() == 1) cout << "Access to file obtained" << endl;
		else cout << "Access to file refused" << endl;
		for (int i = 0; i < N; i++)
		{
			OGmap[i] = new double[N];
			logoddsmap[i] = new double[N];
			for (int j = 0; j < N; j++)
			{
				OGmap[i][j] = 0.5;
				logoddsmap[i][j] = 0;
			}
		}
		for (int i = 0; i < M; i++) errcount[i] = 0;
		cout << "Map initialized!" << endl;
	}
	void mapupdate(ArRobot &robot)
	{
		double rx, ry, ox, oy, theta;
		int ri, rj, oi, oj; 
		rx = robot.getX();
		ry = robot.getY();
		theta = robot.getTh();
		theta = degtorad(theta);
		measurement(robot);
		for (int i = 0; i < M; i++)
		{
			a[i] = degtorad(a[i]);
			if (b[i] < 4000)
			{
				ox = rx + b[i]*cos(a[i] + theta);
				oy = ry + b[i]*sin(a[i] + theta);
				transf(rx, ry, ri, rj);
				transf(ox, oy, oi, oj);
				//BresenhamLine(ri, rj, oi, oj, 1);
				myownAlgorithm(ri, rj, oi, oj, rx, ry, theta, b[i], degtorad(startAngle+i*dfi));
				inversebayes(oj, oi, 1);
			}
			if(b[i]>=4000&&errcount[i]==erroracc)
			{
				ox = rx + 3400 * cos(a[i] + theta);		// PROBLEM Z POMIARAMI, errcount liczy zle pomiary
				oy = ry + 3400 * sin(a[i] + theta);
				transf(rx, ry, ri, rj);
				transf(ox, oy, oi, oj);
				//BresenhamLine(ri, rj, oi, oj, 0);
				myownAlgorithm(ri, rj, oi, oj, rx, ry, theta, b[i], a[i]);
			}
			if (b[i] >= 4000 && errcount[i]!= erroracc)
			{
				transf(rx, ry, ri, rj);
				inversebayes(rj, ri, 0);
			}
		}
	} 
	void finalload()
	{
		for (int i = 0; i < N; i++)
		{
			for (int j = 0; j < N; j++)
			{
				OGmap[i][j] = 1 - 1 / (1 + exp(logoddsmap[i][j]));
			}
		}
		cout << "Map completed" << endl;
	}
	void save()
	{
		F << N << endl;
		for (int i = N-1; i >=0; i--)
		{
			for (int j = 0; j < N; j++)
			{
				F.precision(1);
				F << OGmap[i][j] << "\t";
			}
			F << endl;
		}
		cout << "Map saved" << endl;
	}
	~ArMapa()
	{
		F.close();
		//F2.close();
		for (int i = 0; i < N; i++)
		{
			delete[] OGmap[i];
			delete[] logoddsmap[i];
		}
		delete[] logoddsmap;
		delete[] OGmap;
		delete[] a;
		delete[] b;
		delete[] errcount;
	}
};