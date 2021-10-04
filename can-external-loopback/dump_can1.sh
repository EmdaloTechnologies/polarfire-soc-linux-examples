#!/bin/sh

for ((i=0; i<=1568; i+=4));
  do 
	  number=$(( 16#2010d000))
	  addr=$(($number + $i))
	  'devmem2' `printf "0x%8x\n" $addr`;
  done
