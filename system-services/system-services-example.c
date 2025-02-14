// SPDX-License-Identifier: MIT
/*
 * MSS system services example for the Microchip PolarFire SoC
 *
 * Copyright (c) 2021 Microchip Technology Inc. All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void display_output(char *data, unsigned int byte_length)
{
    unsigned int inc;
    char byte = 0;
    char buffer[2048];
	char *bufferp = buffer;

	for (inc = 0; inc < byte_length; inc++)
	{
		if(inc%32 == 0 && inc != 0)
		{
			bufferp += sprintf(bufferp, "\r\n");
		}
		byte = data[32*(1+inc/32) - inc%32 - 1];
		bufferp += sprintf(bufferp, "%02x", byte);
    }

    printf("%s\n", buffer);
}

int main()
{
    char c[4096];
    char chr;
    FILE *fptr;

    for (;/*ever*/;)
    {
        printf("PolarFire SoC system services example program.\r\nPress:\r\n");
        printf("1 - to show the FPGA device serial number\r\n2 - to show the FPGA device digests\r\n3 - continuously output random numbers from the TRNG, until ctrl+c\r\n4 - to request an ECDSA signature\r\ne - to exit this program\r\n");
        chr = getchar();
        getchar();

        switch (chr)
        {
        case '1':
            if ((fptr = fopen("/dev/mpfs_serial_num", "r")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            fscanf(fptr, "%[^\n]", c);
            printf("Icicle kit serial number: %s\n", c);
            fclose(fptr);
            break;
        case '2':
            if ((fptr = fopen("/dev/mpfs_fpga_digest", "r")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            printf("pfsoc fpga digest:\n");
            size_t ret;
            do {
                ret = fread(c, 1, 66, fptr);
                printf("%.*s", ret, c);
            } while (ret == 66);
            if (feof(fptr))
            {
                printf("\n");
            }
            fclose(fptr);
            break;
        case '3':
            if ((fptr = fopen("/dev/hwrng", "r")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            for (;/*ever*/;)
            {
                fread(c, 32, 1, fptr);
                display_output(c, 32);
            }
            fclose(fptr);
            break;
        case '4':
            if (access("/dev/mpfs_signature", F_OK))
            {
                printf("Error! file doesnt exist\n");
                exit(1);
            }
            else if ((fptr = fopen("/dev/mpfs_signature", "w")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            fprintf(fptr, "47f05d367b0c32e438fb63e6cf4a5f35c2aa2f90dc7543f8");
            fclose(fptr);
            if ((fptr = fopen("/dev/mpfs_signature", "r")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            fread(c, 207, 1, fptr); //3 for status and a space, 208 chars for the DER format hex signature (104 bytes)
            printf("status signature:\r\n%.207s\r\n", c);
            fclose(fptr);
            break;
        case 'd':
            if ((fptr = fopen("/dev/mpfs_fpga_digest", "r")) == NULL)
            {
                printf("Error! opening file\n");
                exit(1);
            }
            fseek(fptr, 50, SEEK_SET);
            fscanf(fptr, "%[^\n]", c);
            printf("Icicle kit serial number: %s\n", c);
            fclose(fptr);
            break;
        case 'e':
            return 0;

        default:
            break;
        }
    }
    return 0;
}