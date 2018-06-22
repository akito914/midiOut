

#include "math.h"

const float sin_table[360] = {0.000000f, 0.017452f, 0.034899f, 0.052336f, 0.069756f, 0.087156f, 0.104528f, 0.121869f, 0.139173f, 0.156434f, 0.173648f, 0.190809f, 0.207912f, 0.224951f, 0.241922f, 0.258819f, 0.275637f, 0.292372f, 0.309017f, 0.325568f, 0.342020f, 0.358368f, 0.374607f, 0.390731f, 0.406737f, 0.422618f, 0.438371f, 0.453990f, 0.469472f, 0.484810f, 0.500000f, 0.515038f, 0.529919f, 0.544639f, 0.559193f, 0.573576f, 0.587785f, 0.601815f, 0.615661f, 0.629320f, 0.642788f, 0.656059f, 0.669131f, 0.681998f, 0.694658f, 0.707107f, 0.719340f, 0.731354f, 0.743145f, 0.754710f, 0.766044f, 0.777146f, 0.788011f, 0.798636f, 0.809017f, 0.819152f, 0.829038f, 0.838671f, 0.848048f, 0.857167f, 0.866025f, 0.874620f, 0.882948f, 0.891007f, 0.898794f, 0.906308f, 0.913545f, 0.920505f, 0.927184f, 0.933580f, 0.939693f, 0.945519f, 0.951057f, 0.956305f, 0.961262f, 0.965926f, 0.970296f, 0.974370f, 0.978148f, 0.981627f, 0.984808f, 0.987688f, 0.990268f, 0.992546f, 0.994522f, 0.996195f, 0.997564f, 0.998630f, 0.999391f, 0.999848f, 1.000000f, 0.999848f, 0.999391f, 0.998630f, 0.997564f, 0.996195f, 0.994522f, 0.992546f, 0.990268f, 0.987688f, 0.984808f, 0.981627f, 0.978148f, 0.974370f, 0.970296f, 0.965926f, 0.961262f, 0.956305f, 0.951057f, 0.945519f, 0.939693f, 0.933580f, 0.927184f, 0.920505f, 0.913545f, 0.906308f, 0.898794f, 0.891007f, 0.882948f, 0.874620f, 0.866025f, 0.857167f, 0.848048f, 0.838671f, 0.829038f, 0.819152f, 0.809017f, 0.798636f, 0.788011f, 0.777146f, 0.766044f, 0.754710f, 0.743145f, 0.731354f, 0.719340f, 0.707107f, 0.694658f, 0.681998f, 0.669131f, 0.656059f, 0.642788f, 0.629320f, 0.615661f, 0.601815f, 0.587785f, 0.573576f, 0.559193f, 0.544639f, 0.529919f, 0.515038f, 0.500000f, 0.484810f, 0.469472f, 0.453990f, 0.438371f, 0.422618f, 0.406737f, 0.390731f, 0.374607f, 0.358368f, 0.342020f, 0.325568f, 0.309017f, 0.292372f, 0.275637f, 0.258819f, 0.241922f, 0.224951f, 0.207912f, 0.190809f, 0.173648f, 0.156434f, 0.139173f, 0.121869f, 0.104528f, 0.087156f, 0.069756f, 0.052336f, 0.034899f, 0.017452f, 0.000000f, -0.017452f, -0.034899f, -0.052336f, -0.069756f, -0.087156f, -0.104528f, -0.121869f, -0.139173f, -0.156434f, -0.173648f, -0.190809f, -0.207912f, -0.224951f, -0.241922f, -0.258819f, -0.275637f, -0.292372f, -0.309017f, -0.325568f, -0.342020f, -0.358368f, -0.374607f, -0.390731f, -0.406737f, -0.422618f, -0.438371f, -0.453990f, -0.469472f, -0.484810f, -0.500000f, -0.515038f, -0.529919f, -0.544639f, -0.559193f, -0.573576f, -0.587785f, -0.601815f, -0.615661f, -0.629320f, -0.642788f, -0.656059f, -0.669131f, -0.681998f, -0.694658f, -0.707107f, -0.719340f, -0.731354f, -0.743145f, -0.754710f, -0.766044f, -0.777146f, -0.788011f, -0.798636f, -0.809017f, -0.819152f, -0.829038f, -0.838671f, -0.848048f, -0.857167f, -0.866025f, -0.874620f, -0.882948f, -0.891007f, -0.898794f, -0.906308f, -0.913545f, -0.920505f, -0.927184f, -0.933580f, -0.939693f, -0.945519f, -0.951057f, -0.956305f, -0.961262f, -0.965926f, -0.970296f, -0.974370f, -0.978148f, -0.981627f, -0.984808f, -0.987688f, -0.990268f, -0.992546f, -0.994522f, -0.996195f, -0.997564f, -0.998630f, -0.999391f, -0.999848f, -1.000000f, -0.999848f, -0.999391f, -0.998630f, -0.997564f, -0.996195f, -0.994522f, -0.992546f, -0.990268f, -0.987688f, -0.984808f, -0.981627f, -0.978148f, -0.974370f, -0.970296f, -0.965926f, -0.961262f, -0.956305f, -0.951057f, -0.945519f, -0.939693f, -0.933580f, -0.927184f, -0.920505f, -0.913545f, -0.906308f, -0.898794f, -0.891007f, -0.882948f, -0.874620f, -0.866025f, -0.857167f, -0.848048f, -0.838671f, -0.829038f, -0.819152f, -0.809017f, -0.798636f, -0.788011f, -0.777146f, -0.766044f, -0.754710f, -0.743145f, -0.731354f, -0.719340f, -0.707107f, -0.694658f, -0.681998f, -0.669131f, -0.656059f, -0.642788f, -0.629320f, -0.615661f, -0.601815f, -0.587785f, -0.573576f, -0.559193f, -0.544639f, -0.529919f, -0.515038f, -0.500000f, -0.484810f, -0.469472f, -0.453990f, -0.438371f, -0.422618f, -0.406737f, -0.390731f, -0.374607f, -0.358368f, -0.342020f, -0.325568f, -0.309017f, -0.292372f, -0.275637f, -0.258819f, -0.241922f, -0.224951f, -0.207912f, -0.190809f, -0.173648f, -0.156434f, -0.139173f, -0.121869f, -0.104528f, -0.087156f, -0.069756f, -0.052336f, -0.034899f, -0.017452f};

float sin_t(float rad){
	int num = 0;

	if(rad < 0.0){
		rad *= -1;
		rad = fmod(rad, 2 * M_PI);
		rad = 6.2831853 - rad;
	}
	else{
		rad = fmod(rad, 2 * M_PI);
	}

	num = (int)(rad * 57.295779513);

	return sin_table[num];

}

float sin_t_e4(int rad_e4){
	int num = 0;

	if(rad_e4 < 0){
		rad_e4 *= -1;
		rad_e4 = rad_e4 % 62831;
		rad_e4 = 62831 - rad_e4;
	}
	else{
		rad_e4 = rad_e4 % 62831;
	}

	num = (int)(rad_e4 * 0.0057295779513);

	return sin_table[num];

}

float sin_t_e5(int rad_e5){
	int num = 0;

	if(rad_e5 < 0){
		rad_e5 *= -1;
		rad_e5 = rad_e5 % 628318;
		rad_e5 = 628318 - rad_e5;
	}
	else{
		rad_e5 = rad_e5 % 628318;
	}

	num = (int)(rad_e5 * 0.00057295779513);

	return sin_table[num];

}
