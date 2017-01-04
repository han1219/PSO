function arr_fitness=init_fitness(pos_present,pop_size,Dim_size)
for k=1:pop_size %初始化每个粒子的适应度
    arr_fitness(k,1) = fitness(pos_present(k,1:Dim_size));  %计算原始种群的适应度
end