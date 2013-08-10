#define CRC32_POLYNOMIAL 0xEDB88320L


namespace novatel {

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for ( j = 8 ; j > 0; j-- ) {
    if ( ulCRC & 1 )
      ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
    else
      ulCRC >>= 1;
  }
    return ulCRC;
}


/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32 ( unsigned long ulCount, /* Number of bytes in the data block */
                                    unsigned char *ucBuffer ) /* Data block */
{
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while ( ulCount-- != 0 ) {
    ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
    ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return( ulCRC );
}

} // end namespace