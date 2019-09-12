# HIROJIREN
code based on anago2018
コンパイル方法
make app=./作業フォルダ/Codes
merge test

--memo--
#include <iostream>
#include <inttypes.h>
#include <stdio.h>


bool twice( int* a )
{
  //   if( a == nullptr )
  if( a == NULL )
    {
      std::cout << "null" << std::endl;
      return true;
    }
  std::cout << *a << std::endl;
  *a *= 2;
  std::cout << *a << std::endl;
  return false;
}

int response_gen(uint8_t **response){
  uint8_t *data;
  const int SIZE = 25;
  uint8_t uint8_dat;
  int i;

  uint8_dat = 0x00;
  std::cout << uint8_dat << std::endl;

  //  printf("%" PRIx8 "\n", uint8_dat);


  data = new uint8_t[SIZE];

  for (i=0; i < SIZE; i++){
    data[i] = uint8_dat;
    uint8_dat = uint8_dat + 0x01;
    //    printf("%" PRIx8 "\n", data[i]);

  }

  *response = data;

  return SIZE;
}

int rx_response(uint8_t *data, size_t data_len)
{
  int i;

  if (data == NULL){
    return -1;
  }

  for (i=0; i < data_len; i++){
    printf("%" PRIx8 "\n", data[i]);
    if(data[i] == 0x0f){
      std::cout <<"**"<< std::endl;
    }

  }


  return 0;
}


int main()
{

  uint8_t *response = NULL;
  int response_len;
  int i;

  response_len = response_gen(&response);
  printf("%" PRIx8 "\n", response[24]); // 20

  std::cout << response_len << std::endl;

  /*
  for (i=0; i < response_len; i++){
    printf("%" PRIx8 "\n", response[i]);
    }*/

  rx_response(response, response_len);


}

