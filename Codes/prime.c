#include <stdio.h>
#include <math.h>
int main()
{
  long int i, j, z;
  long int n;
  long int prime[1000000];
  int      num_p;
    
  /* 判定フラグ */
  int flag;
    
  scanf("%ld", &n);

  /* 素数かどうかを判定 */
    
  prime[0] = 2;
  prime[1] = 3;
  prime[2] = 5;
  prime[3] = 7;
  prime[4] = 11;
  
  num_p =1;


  for( i=3;i<=n;i+=2 ){
      flag = 0;
      printf("i:%ld\n", i);
      for (z = sqrt(i), j = 1; j < num_p && prime[j] <= z; j++){
	printf("i, z, j, num_p, primt_j :%ld, %ld, %ld, %d, %ld\n",i, z, j, num_p, prime[j]);
	if(i%prime[j] == 0){
	  flag = 1;
	  break;
	}
      }
        
      //判定結果を出力
      if( flag==0){
	printf("prime:%ld\n",i);
	prime[num_p] = i;
	num_p        = num_p + 1;
      }
  }


  if( n < 100){
    for (i = 0; i < num_p; i++){
      printf(" %ld",prime[i]);
    }
    printf("\n");
  }else{
    printf("%d\n%ld\n", num_p, prime[num_p-1]);
  }


    
  return 0;
}
