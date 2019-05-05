# make　とすると下記を実行する
# ターゲット : 依存ファイル

test_codes: test_codes.o yawrate_ctl.o
	g++ -Wall -o test_codes test_codes.o yawrate_ctl.o 

yawrate_ctl.o: yawrate_ctl.cpp
	g++ -Wall -c yawrate_ctl.cpp 


test.o: test_codes.cpp
	g++ -Wall -c  test_codes.cpp


# make clean と打つと下記が実行される
clean:
	rm -f *.o *~