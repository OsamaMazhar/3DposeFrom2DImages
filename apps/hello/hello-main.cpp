/*      File: hello-main.cpp
*       This file is part of the program pose-estimation
*       Program description : TODO:inputashortdescriptionofpackagepose-estimationutilityhere
*       Copyright (C) 2018 -  osama (). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the CeCILL license as published by
*       the CEA CNRS INRIA, either version 2.1
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       CeCILL License for more details.
*
*       You should have received a copy of the CeCILL License
*       along with this software. If not, it can be found on the official website
*       of the CeCILL licenses family (http://www.cecill.info/index.en.html).
*/
#include <hello.h>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){
    if(argc == 1){
	cout<<"Only one argument ... error !! Please input a string as argument of the program"<<endl;
	return 1;
    }
    print_Hello(argv[1]);
    return 0;
}
