// eFile.c
// Runs on either TM4C123 or MSP432
// High-level implementation of the file system implementation.
// Daniel and Jonathan Valvano
// August 29, 2016
#include <stdint.h>
#include "eDisk.h"

#define EOF 0xFF
#define NULLFILE 0xFF
#define END_OF_DIRECTORY 0xFF
#define END_OF_FAT 0xFF
#define FILE_READ_ERR 0xFF

uint8_t Buff[512]; // temporary buffer used during file I/O
uint8_t Directory[256], FAT[256];
int32_t bDirectoryLoaded =0; // 0 means disk on ROM is complete, 1 means RAM version active

// Return the larger of two integers.
int16_t max(int16_t a, int16_t b){
  if(a > b){
    return a;
  }
  return b;
}
//*****MountDirectory******
// if directory and FAT are not loaded in RAM,
// bring it into RAM from disk
void MountDirectory(void){ 
// if bDirectoryLoaded is 0, 
//    read disk sector 255 and populate Directory and FAT
//    set bDirectoryLoaded=1
// if bDirectoryLoaded is 1, simply return
  uint8_t disk_read_status;
  uint16_t i;

  if (bDirectoryLoaded == 1)
    return;

  disk_read_status = eDisk_ReadSector(Buff, 255);
  if (disk_read_status != RES_OK)
    return;

  for (i = 0; i < 256; i++) {
    Directory[i] = Buff[i];
    FAT[i] = Buff[i+256];
  }

  bDirectoryLoaded = 1;
}

// Return the index of the last sector in the file
// associated with a given starting sector.
// Note: This function will loop forever without returning
// if the file has no end (i.e. the FAT is corrupted).
uint8_t lastsector(uint8_t start){
  uint8_t last, next;

  last = start;
  while (FAT[last] != EOF) {    // Assumes FAT[255] = 255 (EOF)
    next = FAT[last];
    last = next;
  }
  
  return last; 
}

// Return the index of the first free sector.
// Note: This function will loop forever without returning
// if a file has no end or if (Directory[255] != 255)
// (i.e. the FAT is corrupted).
uint8_t findfreesector(void){
  uint8_t file,
          filenum,
          eof,
          freesector;

  freesector = 0;
  filenum = 0;
  file = Directory[filenum];
  while (file != NULLFILE) {
    eof = lastsector(file);
    freesector = (uint8_t) max(freesector, eof+1);
    file = Directory[++filenum];
  }
  
  return freesector;
}

// Append a sector index 'n' at the end of file 'num'.
// This helper function is part of OS_File_Append(), which
// should have already verified that there is free space,
// so it always returns 0 (successful).
// Note: This function will loop forever without returning
// if the file has no end (i.e. the FAT is corrupted).
uint8_t appendfat(uint8_t num, uint8_t n){
  uint8_t file, eof;

  file = Directory[num];
  if (file == NULLFILE) {   // appending to new file
    Directory[num] = n;
  } else {                  // append to existing file 
    eof = lastsector(file);
    FAT[eof] = n;
  }
	
  return 0;
}

//********OS_File_New*************
// Returns a file number of a new file for writing
// Inputs: none
// Outputs: number of a new file
// Errors: return 255 on failure or disk full
uint8_t OS_File_New(void){
  uint8_t filenum;

  MountDirectory();

  filenum = 0;
  while (1) {
    if (filenum == END_OF_DIRECTORY)
      break;
    if (Directory[filenum] == NULLFILE)
      break;
    filenum++;
  }
	
  return filenum;
}

//********OS_File_Size*************
// Check the size of this file
// Inputs:  num, 8-bit file number, 0 to 254
// Outputs: 0 if empty, otherwise the number of sectors
// Errors:  none
uint8_t OS_File_Size(uint8_t num){
  uint8_t count,
          sector,
          next;

  sector = Directory[num];
  if (sector == 255)
    return 0;

  count = 1;
  while (FAT[sector] != EOF) {
    count++;
    next = FAT[sector];
    sector = next;
  }
	
  return count;
}

//********OS_File_Append*************
// Save 512 bytes into the file
// Inputs:  num, 8-bit file number, 0 to 254
//          buf, pointer to 512 bytes of data
// Outputs: 0 if successful
// Errors:  255 on failure or disk full
uint8_t OS_File_Append(uint8_t num, uint8_t buf[512]){
  uint8_t new_sector;

  new_sector = findfreesector();
  if (new_sector == END_OF_FAT)         // disk is full
    return END_OF_FAT;

  eDisk_WriteSector(buf, new_sector);   // write to disk
  appendfat(num, new_sector);           // updates FAT and Directory as required
  
  return 0;
}

//********OS_File_Read*************
// Read 512 bytes from the file
// Inputs:  num, 8-bit file number, 0 to 254
//          location, logical address, 0 to 254
//          buf, pointer to 512 empty spaces in RAM
// Outputs: 0 if successful
// Errors:  255 on failure because no data
uint8_t OS_File_Read(uint8_t num, uint8_t location,
                     uint8_t buf[512]){
  uint8_t size,
          disk_addr,
          i,
          file_read_status,
          disk_read_status;

  size = OS_File_Size(num);
  if (size == 0 || location >= size)
    file_read_status = FILE_READ_ERR;
  
  i = 0;
  disk_addr = Directory[num];
  while (i < location) {
    disk_addr = FAT[disk_addr];
    i++;
  }

  disk_read_status = eDisk_ReadSector(buf, disk_addr);
  if (disk_read_status != RES_OK)
    file_read_status = FILE_READ_ERR;

  return file_read_status;
}

//********OS_File_Flush*************
// Update working buffers onto the disk
// Power can be removed after calling flush
// Inputs:  none
// Outputs: 0 if success
// Errors:  255 on disk write failure
uint8_t OS_File_Flush(void){
  uint16_t disk_write_status,
           i; 

  if (bDirectoryLoaded == 0)
    return 255;

  for (i = 0; i < 256; i++) {
    Buff[i] = Directory[i];
    Buff[i+256] = FAT[i];
  }
 
  disk_write_status = eDisk_WriteSector(Buff, 255);

  return (disk_write_status != RES_OK) ? 255 : 0;
}

//********OS_File_Format*************
// Erase all files and all data
// Inputs:  none
// Outputs: 0 if success
// Errors:  255 on disk write failure
uint8_t OS_File_Format(void){
// call eDiskFormat
// clear bDirectoryLoaded to zero
  uint8_t format_status;

  format_status = eDisk_Format();
  bDirectoryLoaded = 0;

  return (format_status != RES_OK) ? 255: 0;
}
