function [pso F] = pso_2D()
clear;
clc;
%   FUNCTION PSO  --------USE Particle Swarm Optimization Algorithm
%global present;
% close all;
pop_size = 100;          %   pop_size 种群大小
Dim_size = 2;          %   Dim_size 粒子大小,                                                                      ** =n-D
gbest = zeros(1,Dim_size+1);            %   gbest 当前搜索到的最小的值
pbest = zeros(pop_size,Dim_size+1);      %   pbest 粒子以前搜索到的最优值，最后一列包括这些值的适应度
max_gen = 80;          %   max_gen 最大迭代次数
region=zeros(Dim_size,2);  %   设定搜索空间范围
region=[-3,3;-3,3];          %                                                                                      **每一维设定不同范围

isMinFlag=1;
rand('state',sum(100*clock));   %   重置随机数发生器状态
arr_present = ini_pos(pop_size,Dim_size);   %   present 当前位置,随机初始化,rand()的范围为0~1
v=ini_v(pop_size,Dim_size);             %   初始化当前速度

w_max = 0.9;                            %   w_max 权系数最大值
w_min = 0.4;
v_max = 2;             %                                                                                          **最大速度,为粒子的范围宽度
c1 = 2;                   %   学习因子
c2 = 2;                   %   学习因子
best_record = zeros(1,max_gen);     %   best_record记录最好的粒子的适应度位于第几代。
%  ————————————————————————
%   计算原始种群的适应度,及初始化
%  ————————————————————————
arr_present(:,end)=init_fitness(arr_present,pop_size,Dim_size);


% for k=1:pop_size
%     present(k,end) = fitness(present(k,1:Dim_size));  %计算原始种群的适应度
% end

pbest = arr_present;                                        %初始化各个粒子最优值
[best_value best_index] = min(arr_present(:,end));         %初始化全局最优，即适应度为全局最小的值，根据需要也可以选取为最大值
gbest = arr_present(best_index,:);

%v = zeros(pop_size,1);          %   v 速度
%  ————————————————————————
%   迭代
%  ————————————————————————
% global m;
% m = moviein(1000);      %生成帧矩阵
x=[-3:0.01:3];
y=[-3:0.01:3];
z=@(x,y) 3*(1-x).^2.*exp(-(x.^2) - (y+1).^2) ...
    - 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2) ...
    - 1/3*exp(-(x+1).^2 - y.^2);
for i=1:max_gen
    grid on;
    ezmesh(z),hold on,grid on,plot3(arr_present(:,1),arr_present(:,2),arr_present(:,3),'*'),hold off;
    drawnow
    F(i)=getframe;
    pause(0.01);
    %     m(:,i) = getframe;        %添加图形
    w = w_max-(w_max-w_min)*i/max_gen;
    %    fprintf('#  %i 代开始！\n',i);
    %   确定是否对打散已经收敛的粒子群——————————————————————————————
    reset = 0;          %   reset = 1时设置为粒子群过分收敛时将其打散，如果＝1则不打散
%     if reset==1
%         bit = 1;
%         for k=1:Dim_size
%             bit = bit&(range(arr_present(:,k))<0.1);
%         end
%         if bit==1       %   bit=1时对粒子位置及速度进行随机重置
%             arr_present = ini_pos(pop_size,Dim_size);   %   present 当前位置,随机初始化
%             v = ini_v(pop_size,Dim_size);           %   速度初始化
%             for k=1:pop_size                                    %   重新计算适应度
%                 arr_present(k,end) = fitness(arr_present(k,1:Dim_size));
%             end
%             warning('粒子过分集中！重新初始化……');      %   给出信息
%             display(i);
%         end
%     end

    for j=1:pop_size
        v(j,:) = w.*v(j,:)+c1.*rand.*(pbest(j,1:Dim_size)-arr_present(j,1:Dim_size))...
            +c2.*rand.*(gbest(1:Dim_size)-arr_present(j,1:Dim_size));                        %  粒子速度更新 (a)

        %   判断v的大小，限制v的绝对值小于5————————————————————————————
        c = find(abs(v)>6);   %找出所有速度大于6的                                                                                           %**最大速度设置，粒子的范围宽度
        v(c) = sign(v(c))*6;   %如果速度大于3.14则，速度为3.14

        arr_present(j,1:Dim_size) = arr_present(j,1:Dim_size)+v(j,1:Dim_size);              %  粒子位置更新 (b)
        arr_present(j,end) = fitness(arr_present(j,1:Dim_size));

        if ( arr_present(j,end)<pbest(j,end))&(Region_in(arr_present(j,:),region))     %   根据条件更新pbest,如果是最小的值为小于号,相反则为大于号
            pbest(j,:) = arr_present(j,:);
        end

    end

    [best best_index] = min(arr_present(:,end));                                      %   如果是最小的值为min,相反则为max

    if best<gbest(end)&(Region_in(arr_present(best_index,:),region))                  %   如果当前最好的结果比以前的好，则更新最优值gbest,如果是最小的值为小于号,相反则为大于号
        gbest = arr_present(best_index,:);
    end

    best_record(i) = gbest(end);

end

pso = gbest;

display(gbest);

% figure;
% plot(best_record);
% movie2avi(F,'pso_2D1.avi','compression','MSVC');



function flag=Region_in(pos_present,region)
[m n]=size(pos_present);
flag=1;
for j=1:n-1
    flag=flag&(pos_present(1:j)>=region(j,1))&(pos_present(1:j)<=region(j,2));
end
