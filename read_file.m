% Written by Muhammet Balcilar, France
% All rights reserved

clear all;
%close all
clc;

image_2_path = 'data/image_2/';
calib_path = 'data/calib/';
bin_path = 'data/velodyne/';
File = dir(fullfile(image_2_path,'*.png'));  % ��ʾ�ļ��������з��Ϻ�׺��Ϊ.png�ļ���������Ϣ
FileNames = {File.name}';                    % ��ȡ���Ϻ�׺��Ϊ.txt�������ļ����ļ�����ת��Ϊn��1��
FileNumbers = size(FileNames);
FileNumbers = FileNumbers(1);

for i=1:FileNumbers
    file_name = char(FileNames(i));
    temp_ = strsplit(file_name,'.');
    file_id = char(temp_(1));
end
