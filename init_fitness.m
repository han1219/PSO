function arr_fitness=init_fitness(pos_present,pop_size,Dim_size)
for k=1:pop_size %��ʼ��ÿ�����ӵ���Ӧ��
    arr_fitness(k,1) = fitness(pos_present(k,1:Dim_size));  %����ԭʼ��Ⱥ����Ӧ��
end