function [pso F] = pso_2D()
clear;
clc;
%   FUNCTION PSO  --------USE Particle Swarm Optimization Algorithm
%global present;
% close all;
pop_size = 100;          %   pop_size ��Ⱥ��С
Dim_size = 2;          %   Dim_size ���Ӵ�С,                                                                      ** =n-D
gbest = zeros(1,Dim_size+1);            %   gbest ��ǰ����������С��ֵ
pbest = zeros(pop_size,Dim_size+1);      %   pbest ������ǰ������������ֵ�����һ�а�����Щֵ����Ӧ��
max_gen = 80;          %   max_gen ����������
region=zeros(Dim_size,2);  %   �趨�����ռ䷶Χ
region=[-3,3;-3,3];          %                                                                                      **ÿһά�趨��ͬ��Χ

isMinFlag=1;
rand('state',sum(100*clock));   %   ���������������״̬
arr_present = ini_pos(pop_size,Dim_size);   %   present ��ǰλ��,�����ʼ��,rand()�ķ�ΧΪ0~1
v=ini_v(pop_size,Dim_size);             %   ��ʼ����ǰ�ٶ�

w_max = 0.9;                            %   w_max Ȩϵ�����ֵ
w_min = 0.4;
v_max = 2;             %                                                                                          **����ٶ�,Ϊ���ӵķ�Χ���
c1 = 2;                   %   ѧϰ����
c2 = 2;                   %   ѧϰ����
best_record = zeros(1,max_gen);     %   best_record��¼��õ����ӵ���Ӧ��λ�ڵڼ�����
%  ������������������������������������������������
%   ����ԭʼ��Ⱥ����Ӧ��,����ʼ��
%  ������������������������������������������������
arr_present(:,end)=init_fitness(arr_present,pop_size,Dim_size);


% for k=1:pop_size
%     present(k,end) = fitness(present(k,1:Dim_size));  %����ԭʼ��Ⱥ����Ӧ��
% end

pbest = arr_present;                                        %��ʼ��������������ֵ
[best_value best_index] = min(arr_present(:,end));         %��ʼ��ȫ�����ţ�����Ӧ��Ϊȫ����С��ֵ��������ҪҲ����ѡȡΪ���ֵ
gbest = arr_present(best_index,:);

%v = zeros(pop_size,1);          %   v �ٶ�
%  ������������������������������������������������
%   ����
%  ������������������������������������������������
% global m;
% m = moviein(1000);      %����֡����
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
    %     m(:,i) = getframe;        %���ͼ��
    w = w_max-(w_max-w_min)*i/max_gen;
    %    fprintf('#  %i ����ʼ��\n',i);
    %   ȷ���Ƿ�Դ�ɢ�Ѿ�����������Ⱥ������������������������������������������������������������
    reset = 0;          %   reset = 1ʱ����Ϊ����Ⱥ��������ʱ�����ɢ�������1�򲻴�ɢ
%     if reset==1
%         bit = 1;
%         for k=1:Dim_size
%             bit = bit&(range(arr_present(:,k))<0.1);
%         end
%         if bit==1       %   bit=1ʱ������λ�ü��ٶȽ����������
%             arr_present = ini_pos(pop_size,Dim_size);   %   present ��ǰλ��,�����ʼ��
%             v = ini_v(pop_size,Dim_size);           %   �ٶȳ�ʼ��
%             for k=1:pop_size                                    %   ���¼�����Ӧ��
%                 arr_present(k,end) = fitness(arr_present(k,1:Dim_size));
%             end
%             warning('���ӹ��ּ��У����³�ʼ������');      %   ������Ϣ
%             display(i);
%         end
%     end

    for j=1:pop_size
        v(j,:) = w.*v(j,:)+c1.*rand.*(pbest(j,1:Dim_size)-arr_present(j,1:Dim_size))...
            +c2.*rand.*(gbest(1:Dim_size)-arr_present(j,1:Dim_size));                        %  �����ٶȸ��� (a)

        %   �ж�v�Ĵ�С������v�ľ���ֵС��5��������������������������������������������������������
        c = find(abs(v)>6);   %�ҳ������ٶȴ���6��                                                                                           %**����ٶ����ã����ӵķ�Χ���
        v(c) = sign(v(c))*6;   %����ٶȴ���3.14���ٶ�Ϊ3.14

        arr_present(j,1:Dim_size) = arr_present(j,1:Dim_size)+v(j,1:Dim_size);              %  ����λ�ø��� (b)
        arr_present(j,end) = fitness(arr_present(j,1:Dim_size));

        if ( arr_present(j,end)<pbest(j,end))&(Region_in(arr_present(j,:),region))     %   ������������pbest,�������С��ֵΪС�ں�,�෴��Ϊ���ں�
            pbest(j,:) = arr_present(j,:);
        end

    end

    [best best_index] = min(arr_present(:,end));                                      %   �������С��ֵΪmin,�෴��Ϊmax

    if best<gbest(end)&(Region_in(arr_present(best_index,:),region))                  %   �����ǰ��õĽ������ǰ�ĺã����������ֵgbest,�������С��ֵΪС�ں�,�෴��Ϊ���ں�
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
