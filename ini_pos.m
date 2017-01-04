function ini_present=ini_pos(pop_size,Dim_size)
ini_present = 3*rand(pop_size,Dim_size+1);        %初始化当前粒子位置,使其随机的分布在工作空间                         %** 6即为自变量范围
