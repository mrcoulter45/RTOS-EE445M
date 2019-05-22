/**
 * @file      eFile.c
 * @brief     high-level file system
 * @details   This file system sits on top of eDisk.
 * @version   V1.0
 * @author    Jake Klovenski, Michael Coulter

 Based on eFile.h written by Valvano
 * @copyright Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      March 9, 2017

 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "eDisk.h"
#include "UART.h"
#include "eFile.h"
// Struct for each file to be represented in RAM
// A file struct is 12 bytes, meaning  42 will fit in a single block
// We are required to support at minimum 10 file so one block of file allocation is perfect
struct file {
  char name[8]; //7 characters plus null terminate
  uint16_t size;  // how many blocks does it take up
  uint16_t block; // the first block
};
typedef struct file fileType;

#define FILE_SIZE 12
#define BLOCK_SIZE 512
#define FAT_BLOCKS 4
#define NUM_FILE_BLOCKS 1
#define MAX_BLOCK 2048
#define NUM_FILES_PER_BLOCK BLOCK_SIZE/FILE_SIZE

uint8_t fileBuffer[BLOCK_SIZE];
fileType files[NUM_FILE_BLOCKS * NUM_FILES_PER_BLOCK];
uint8_t FAT[FAT_BLOCKS*BLOCK_SIZE];

uint16_t file_open;
uint16_t block_open;
uint16_t reading;
uint16_t reading_index;
uint16_t redirect;
/**
 * @details This function must be called first, before calling any of the other eFile functions
 * @param  none
 * @return 0 if successful and 1 on failure (already initialized)
 * @brief  Activate the file system, without formating
 */
int eFile_Init(void) // initialize file system
{
  if(eDisk_Init(0)) return 1;
  //need to load file data from disk
  for(int i = 0; i < NUM_FILE_BLOCKS; i++)
  {
    if(eDisk_ReadBlock(fileBuffer, i))//need to read a block of data and make sure it is read correctly
    {
      return 1;
    }
    uint8_t* currFile = fileBuffer;
    for(int j = 0; j < NUM_FILES_PER_BLOCK; j++)  // need to load every file meta-data from list
    {
      //need the name
      for(int k = 0; k < 8; k++)
      {
        files[i*NUM_FILES_PER_BLOCK + j].name[k] = currFile[k];
      }
      //need the sizes
      uint16_t fileSize = currFile[8] << 8;
      fileSize += currFile[9];
      files[i*NUM_FILES_PER_BLOCK + j].size = fileSize;
      //need the block
      uint16_t first_block = currFile[10] << 8;
      first_block += currFile[11];
      files[i*NUM_FILES_PER_BLOCK + j].block = first_block;
    }
    file_open = 0;
    block_open = 0;
    reading = 0;
    reading_index = 0;
    redirect = 0;
    return 0;
  }
  //need to load FAT from disk
  if(eDisk_Read(0,FAT,NUM_FILE_BLOCKS,FAT_BLOCKS)) //pretty simple, FAT is guaranteed to be continous in memory and will be after the file metadata
  {
    return 1;
  }
  return 0;
}

uint32_t StoreFiles()
{
  for(int i = 0; i < NUM_FILE_BLOCKS; i++)
  {
    uint8_t *buff = fileBuffer;
    for(int j = 0; j < NUM_FILES_PER_BLOCK; j++)
    {
      for(int k = 0; k < 8; k++)
      {
        buff[k] = files[i*NUM_FILES_PER_BLOCK + j].name[k];
      }
      buff[8] = (files[i*NUM_FILES_PER_BLOCK + j].size >> 8);
      buff[9] = (files[i*NUM_FILES_PER_BLOCK + j].size);
      buff[10] = (files[i*NUM_FILES_PER_BLOCK + j].block >> 8);
      buff[11] = (files[i*NUM_FILES_PER_BLOCK + j].block);
      buff+=FILE_SIZE;
    }
    if(eDisk_WriteBlock(fileBuffer,i))
    {
      return 1;
    }
  }
  return 0;
}

uint32_t storeFAT()
{
  if(eDisk_Write(0,FAT,NUM_FILE_BLOCKS, FAT_BLOCKS))
  {
    return 1;
  }
  return 0;
}
/**
 * @details Erase all files, create blank directory, initialize free space manager
 * @param  none
 * @return 0 if successful and 1 on failure (e.g., trouble writing to flash)
 * @brief  Format the disk
 */
int eFile_Format(void) // erase disk, add format
{
  //clear files
  for(int i = 0; i < NUM_FILE_BLOCKS; i++)
  {
    for(int j = 0; j < NUM_FILES_PER_BLOCK; j++)
    {
      files[i*NUM_FILES_PER_BLOCK + j].name[0] = 0;
      files[i*NUM_FILES_PER_BLOCK + j].size = 0;
      files[i*NUM_FILES_PER_BLOCK + j].block = 0;
    }
  }
  //free space
  files[0].name[0] = '*';
  files[0].name[1] = 0;
  files[0].block = NUM_FILE_BLOCKS*NUM_FILES_PER_BLOCK + FAT_BLOCKS;
  uint32_t res = StoreFiles();
  //clear FAT
  for(int i = 0; i < NUM_FILE_BLOCKS*NUM_FILES_PER_BLOCK + FAT_BLOCKS; i++)
  {
    FAT[i] = 0; //files and fat blocks should not be set to anything
  }
  for(int i = NUM_FILE_BLOCKS*NUM_FILES_PER_BLOCK + FAT_BLOCKS; i < MAX_BLOCK; i++)
  {
    FAT[i] = i + 1; //set data element of the FAT to the next value
  }
  FAT[MAX_BLOCK - 1] = 0; //last one shouldn't point to anything
  res+=storeFAT();
  return res;
}
/**
 * Finds a file by the name given, if found returns its index, if not returns 0
 */
uint32_t findFileByName(char name[])
{
  for(int i = 1; i < NUM_FILES_PER_BLOCK * NUM_FILE_BLOCKS; i++)
  {
    if(strcmp(files[i].name,name) == 0)
    {
      return i;
    }
  }
  return 0;
}
/**
 * @details Create a new, empty file with one allocated block
 * @param  name file name is an ASCII string up to seven characters
 * @return 0 if successful and 1 on failure (e.g., already exists)
 * @brief  Create a new file
 */
int eFile_Create( char name[])  // create new file, make it empty
{
  if(findFileByName(name))
  {
    return 1;
  }
  uint16_t new_file = 0;
  for(int i = 1; i < NUM_FILES_PER_BLOCK * NUM_FILE_BLOCKS; i++)
  {
    if(files[i].name[0] == 0)
    {
      new_file = i;
      break;
    }
  }
  if(new_file == 0) // no more room for file
  {
    return 1;
  }
  //put name in new file
  for(int i = 0; i < 8; i++)
  {
    files[new_file].name[i] = name[i];
    if(name[i] == 0)
    {
      break;
    }
  }
  //put size as 0 in new file
  files[new_file].size = 0;
  //assign a new block in new file and move the free pointer forward
  uint32_t freeBlock = files[0].block;
  files[0].block = FAT[freeBlock];
  FAT[freeBlock] = 0;
  files[new_file].block = freeBlock;
  //store changes to both FAT and files
  uint32_t res = storeFAT() + StoreFiles();
  return res;
}

uint32_t findLastBlock(uint32_t file_num)
{
  uint32_t block = files[file_num].block;
  while(FAT[block] != 0)
  {
    block = FAT[block];
  }
  return block;
}

/**
 * @details Open the file for writing, read into RAM last block
 * @param  name file name is an ASCII string up to seven characters
 * @return 0 if successful and 1 on failure (e.g., trouble reading from flash)
 * @brief  Open an existing file for writing
 */
int eFile_WOpen(char name[])      // open a file for writing
{
  if(file_open)
  {
    return 1;
  }
  uint32_t file_num = findFileByName(name);
  uint32_t last_block = findLastBlock(file_num);
  if(eDisk_ReadBlock(fileBuffer,last_block) != 0)
  {
    return 1;
  }
  block_open = last_block;
  file_open = file_num;
  reading = 0;
  return 0;
}

/**
 * @details Save one byte at end of the open file
 * @param  data byte to be saved on the disk
 * @return 0 if successful and 1 on failure (e.g., trouble writing to flash)
 * @brief  Format the disk
 */
int eFile_Write(char data)
{
  //Is there a file open
  if(file_open == 0)
  {
    return 1;
  }
  //Is the file open for reading or writing
  if(reading)
  {
    return 1;
  }
  if(files[file_open].size % BLOCK_SIZE == 0 && files[file_open].size > 0) // allocate a new  block if the block is full
  {
    if(eDisk_WriteBlock(fileBuffer, block_open ))
    {
      return 1;
    }
    //Get new open block from FAT
    uint32_t newBlock = files[0].block;
    files[0].block = FAT[newBlock];
    FAT[newBlock] = 0;
    //add the new block to the file
    uint32_t old_last_block = block_open;
    FAT[old_last_block] = newBlock;
    block_open = newBlock;
  }
  uint32_t place = files[file_open].size % BLOCK_SIZE;
  fileBuffer[place] = data;
  files[file_open].size+=1;
  return 0;
}

/**
 * @details Close the file, leave disk in a state power can be removed.
 * This function will flush all RAM buffers to the disk.
 * @param  none
 * @return 0 if successful and 1 on failure (e.g., trouble writing to flash)
 * @brief  Close the file that was being written
 */
int eFile_WClose(void) // close the file for writing
{
  if(file_open == 0) // no file open to close
  {
    return 1;
  }
  if(reading) // not writing if it is being read
  {
    return 1;
  }
  if(eDisk_WriteBlock(fileBuffer, block_open)) //write whatever is in the buffer to memory
  {
    return 1;
  }
  if(storeFAT())  //FAT has probably been changed
  {
    return 1;
  }
  if(StoreFiles())  //again probably changed
  {
    return 1;
  }
  file_open = 0;
  block_open = 0;
  return 0;
}

/**
 * @details Open the file for reading, read first block into RAM
 * @param  name file name is an ASCII string up to seven characters
 * @return 0 if successful and 1 on failure (e.g., trouble reading from flash)
 * @brief  Open an existing file for reading
 */
int eFile_ROpen(char name[])      // open a file for reading
{
  if(file_open != 0)
  {
    return 1;
  }
  reading = 1;
  file_open = findFileByName(name);
  if(file_open == 0)
  {
    return 1;
  }
  block_open = files[file_open].block;
  if(eDisk_ReadBlock(fileBuffer,block_open))
  {
    return 1;
  }
  reading_index = 0;
  return 0;
}

/**
 * @details Read one byte from disk into RAM
 * @param  pt call by reference pointer to place to save data
 * @return 0 if successful and 1 on failure (e.g., trouble reading from flash)
 * @brief  Retreive data from open file
 */
int eFile_ReadNext(char *pt)       // get next byte
{
  if(file_open == 0) //no open file to read from
  {
    return 1;
  }
  if(reading == 0)  //the file is not open for reading
  {
    return 2;
  }
  if(reading_index == files[file_open].size)  //nothing else to read
  {
    return 3;
  }
  if(reading_index%BLOCK_SIZE==0 && reading_index > 0) //load new block if this one has been fully read
  {
    block_open = FAT[block_open]; // just get the next block in the file, nothing was changed by reading it so no need to write
    if(eDisk_ReadBlock(fileBuffer, block_open))
    {
      return 4;
    }
  }
  *pt = fileBuffer[reading_index%BLOCK_SIZE]; //pass next byte by reference
  reading_index+=1;
  return 0;
}

/**
 * @details Close the file, leave disk in a state power can be removed.
 * @param  none
 * @return 0 if successful and 1 on failure (e.g., wasn't open)
 * @brief  Close the file that was being read
 */
int eFile_RClose(void) // close the file for writing
{
  if(file_open == 0) // no file open to close
  {
    return 1;
  }
  if(reading == 0)  // wasn't reading
  {
    return 1;
  }
  //nothing is changed so no need to write anything back to the disk
  file_open = 0;
  block_open = 0;
  return 0;
}

/**
 * @details Deactivate the file system. One can reactive the file system with eFile_Init.
 * @param  none
 * @return 0 if successful and 1 on failure (e.g., trouble writing to flash)
 * @brief  Close the disk
 */
int eFile_Close(void)
{
  if(storeFAT())
  {
    return 1;
  }
  if(StoreFiles())
  {
    return 1;
  }
  //Several conditions could occur with these, but nothing is really lost by calling them
  eFile_WClose();
  eFile_RClose();
  return 0;
}

/**
 * @details Display the directory with filenames and sizes
 * @param  fp pointer to a function that outputs ASCII characters to display
 * @return 0 if successful and 1 on failure (e.g., trouble reading from flash)
 * @brief  Show directory
 */
int eFile_Directory(void(*fp)(char))
{
  for(int i = 1; i < NUM_FILE_BLOCKS * NUM_FILES_PER_BLOCK; i++)
  {
    if(files[i].name[0] != 0) // only print out names of files that matter
    {
      uint8_t name_done = 0;
      for(int j = 0; j < 8; j++)
      {
        if(files[i].name[j] == 0) // keep the files printed out nice and orderly
        {
          name_done = 1;
        }
        if(name_done)
        {
          (*fp)(' ');
        }
        else
        {
          (*fp)(files[i].name[j]);
        }
      }
      (*fp)(' ');
			UART_OutUDec(files[i].size);
      (*fp)(' ');
			(*fp)('\n');
			(*fp)('\r');
      //TODO: Come back here and write the functionality to print size. It is currently 2:13 and I'm tired.
    }
  }
	return 0;
}

/**
 * @details Delete the file with this name, recover blocks so they can be used by another file
 * @param  name file name is an ASCII string up to seven characters
 * @return 0 if successful and 1 on failure (e.g., file doesn't exist)
 * @brief  delete this file
 */
int eFile_Delete(char name[])  // remove this file
{
  //find the file in files
  uint32_t file = findFileByName(name);
  if(file == 0) // file does not exist
  {
    return 1;
  }
  //add blocks from this file back to the open space
  uint16_t first_block_deleted = files[file].block;
  uint16_t last_block_free = findLastBlock(0);
  FAT[last_block_free] = first_block_deleted; //already linked just add to end of free space
  //clear data of the file
  files[file].name[0] = 0;
  files[file].size = 0;
  files[file].block = 0;
  return 0;
}

/**
 * @details open the file for writing, redirect stream I/O (printf) to this file
 * @note if the file exists it will append to the end<br>
 If the file doesn't exist, it will create a new file with the name
 * @param  name file name is an ASCII string up to seven characters
 * @return 0 if successful and 1 on failure (e.g., can't open)
 * @brief  redirect printf output into this file
 */
int eFile_RedirectToFile(char *name)
{
  //Does the file exist
  uint16_t file = findFileByName(name);
  if(file == 0) //If not make the file
  {
    if(eFile_Create(name))
    {
      return 1;
    }
  }
  if(eFile_WOpen(name)) // open the file for writing
  {
    return 1;
  }
  redirect = 1;
  return 0;
}

/**
 * @details close the file for writing, redirect stream I/O (printf) back to the UART
 * @param  none
 * @return 0 if successful and 1 on failure (e.g., trouble writing)
 * @brief  Stop streaming printf to file
 */
int eFile_EndRedirectToFile(void)
{
  if(eFile_WClose())
  {
    return 1;
  }
  redirect = 0;
  return 0;
}

//fputc override, We don't use it for our UART so it doesn't really affect our interpreter but whatever
int fputc(int ch, FILE *f)
{
  if(redirect)
  {
    if(eFile_Write(ch))
    {
      eFile_EndRedirectToFile();
      return 1;
    }
    return 0;
  }
	else
	{
		UART_OutChar(ch);
	}
	return 0;
}
