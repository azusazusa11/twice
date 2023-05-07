#include <cstdio>

#define WRITE(...) fprintf(f, __VA_ARGS__)

static void
generate_arm_lut(FILE *f)
{
	WRITE("#include \"nds/arm/interpreter/lut.h\"\n");
	WRITE("#include \"nds/arm/interpreter/arm_inst.h\"\n\n");
	WRITE("namespace twice {\n\n");

	WRITE("const ArmInstruction arm_inst_lut[4096] = {\n");

	for (unsigned int i = 0; i < 4096; i++) {
		WRITE("\t");

		/* undefined instructions */
		if ((i & 0xFB0) == 0x300) {
			WRITE("arm_undefined");
		} else if ((i & 0xE01) == 0x601) {
			WRITE("arm_undefined");
		}

		/* branch instructions */
		else if ((i & 0xE00) == 0xA00) {
			int L = i >> 8 & 1;
			WRITE("arm_b<%d>", L);
		} else if (i == 0x123) {
			WRITE("arm_blx2");
		} else if (i == 0x121) {
			WRITE("arm_bx");
		}

		/* multiply instructions */
		else if ((i & 0xFCF) == 0x009) {
			int A = i >> 5 & 1;
			int S = i >> 4 & 1;
			WRITE("arm_multiply<%d, %d>", A, S);
		} else if ((i & 0xF8F) == 0x089) {
			int U = i >> 6 & 1;
			int A = i >> 5 & 1;
			int S = i >> 4 & 1;
			WRITE("arm_multiply_long<%d, %d, %d>", U, A, S);
		}

		/* misc arithmetic instructions */
		else if (i == 0x161) {
			WRITE("arm_clz");
		}

		/* status register access instructions */
		else if ((i & 0xFBF) == 0x100) {
			int R = i >> 6 & 1;
			WRITE("arm_mrs<%d>", R);
		} else if ((i & 0xFB0) == 0x320) {
			int R = i >> 6 & 1;
			WRITE("arm_msr<1, %d>", R);
		} else if ((i & 0xFBF) == 0x120) {
			int R = i >> 6 & 1;
			WRITE("arm_msr<0, %d>", R);
		}

		/* exception generating instructions */
		else if ((i & 0xF00) == 0xF00) {
			WRITE("arm_swi");
		} else if (i == 0x127) {
			WRITE("arm_bkpt");
		}

		/* saturated addition and subtraction instructions */
		else if ((i & 0xF9F) == 0x105) {
			int OP = i >> 5 & 3;
			WRITE("arm_sat_add_sub<%d>", OP);
		}

		/* dsp integer multiply and multiply accumulate instructions */
		else if ((i & 0xF99) == 0x108) {
			int OP = i >> 5 & 3;
			int Y = i >> 2 & 1;
			int X = i >> 1 & 1;
			WRITE("arm_dsp_multiply<%d, %d, %d>", OP, Y, X);
		}

		/* semaphore instructions */
		else if ((i & 0xFBF) == 0x109) {
			int B = i >> 6 & 1;
			WRITE("arm_swap<%d>", B);
		}

		/* everything else */
		else {
			WRITE("arm_undefined");
		}

		WRITE(",\n");
	}

	WRITE("};\n\n");
	WRITE("}\n");
}

static void
generate_thumb_lut(FILE *f)
{
	WRITE("#include \"nds/arm/interpreter/lut.h\"\n");
	WRITE("#include \"nds/arm/interpreter/thumb_inst.h\"\n\n");
	WRITE("namespace twice {\n\n");

	WRITE("const ThumbInstruction thumb_inst_lut[1024] = {\n");

	for (unsigned int i = 0; i < 1024; i++) {
		WRITE("\t");
		WRITE("thumb_noop");
		WRITE(",\n");
	}

	WRITE("};\n\n");
	WRITE("}\n");
}

int
main(int argc, char **argv)
{
	(void)argc;

	FILE *f1 = fopen(argv[1], "w");
	if (!f1) {
		fprintf(stderr, "could not open file: %s\n", argv[1]);
		return 1;
	}

	FILE *f2 = fopen(argv[2], "w");
	if (!f2) {
		fprintf(stderr, "could not open file: %s\n", argv[2]);
		return 1;
	}

	generate_arm_lut(f1);
	generate_thumb_lut(f2);

	fclose(f1);
	fclose(f2);

	return 0;
}
