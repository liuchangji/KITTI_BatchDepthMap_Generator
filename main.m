% Written by Muhammet Balcilar, France
% All rights reserved
% LCJ修改，用于生成稀疏深度图像
% 源程序来自于博客：https://blog.csdn.net/linghugoolge/article/details/85222527
% 文件夹结构：
%--Data---|
%         |- calib (标定矩阵文件）
%         |- image_2 (左目图像）
%         |- velodyne (激光雷达）
%         |- depth (手动新建一个存放depth图的文件夹）
%
% data_path,运行该程序，
clear all
clc;
data_path =  'C:/liuchangji/基于实例分割的目标三维位置估计方法/KITTI/object/training/';
image_2_path = [data_path,'image_2/'];
calib_path = [data_path,'calib/'];
bin_path = [data_path,'velodyne/'];
depth_map_save_path = [data_path,'depth/'];
File = dir(fullfile(image_2_path,'*.png'));  % 显示文件夹下所有符合后缀名为.png文件的完整信息
FileNames = {File.name}';                    % 提取符合后缀名为.txt的所有文件的文件名，转换为n行1列
FileNumbers = size(FileNames);
FileNumbers = FileNumbers(1);

for i=1:FileNumbers
    process_information = fprintf('正在处理%d/%d    ',i,FileNumbers);
    file_name = char(FileNames(i));
    temp_ = strsplit(file_name,'.');
    file_id = char(temp_(1));

    I=imread([image_2_path,file_id,'.png']);

    %% read calibration file

    txt_fd = fopen([calib_path,file_id,'.txt']);
    raw_data = fscanf(txt_fd,'%c');
    fclose(txt_fd);

    ii = find(raw_data == ':')+1;
    ie = find(raw_data == 10 );

    P0 = reshape(str2num( raw_data(ii(1):ie(1)) ),4,3)';
    P1 = reshape(str2num( raw_data(ii(2):ie(2)) ),4,3)';
    P2 = reshape(str2num( raw_data(ii(3):ie(3)) ),4,3)';
    P3 = reshape(str2num( raw_data(ii(4):ie(4)) ),4,3)';
    R0_rect = reshape(str2num( raw_data(ii(5):ie(5)) ),3,3)';
    Tr_velo_to_cam = reshape(str2num( raw_data(ii(6):ie(6)) ),4,3)';
    Tr_imu_to_velo = reshape(str2num( raw_data(ii(7):ie(7)) ),4,3)';

    %% read Lidar data file
    bin_fd = fopen([bin_path,file_id,'.bin'],'rb');
    velo = fread(bin_fd,[4 inf],'single')';
    fclose(bin_fd);

    % remove all points behind image plane (approximation)
    idx = velo(:,1)<5;
    velo(idx,:) = [];

    % exclude luminance make last column all 1
    % the first 3 values correspond to x,y and z, and the last value is the reflectance information.
    % x,y and y are stored in metric (m) Velodyne coordinates.

    velo(:,4)=1;

    % draw raw point cloud
%     figure;
%     plot3(velo(:,1),velo(:,2),velo(:,3),'r.');
%     title('Raw point Cloud');


    % create projection matrix
    R0_rect(4,4)=1;
    Tr_velo_to_cam(4,4)=1;
    P=P2 * R0_rect * Tr_velo_to_cam;

    % project to image plane 
    px = (P * velo')';

    px(:,1) = px(:,1)./px(:,3);
    px(:,2) = px(:,2)./px(:,3);

    % remove out of image size indexes
    px(px(:,1)<1,:)=[];
    px(px(:,1)>size(I,2),:)=[];    
    px(px(:,2)>size(I,1),:)=[];

    [n m k]=size(I);

    tic
    depth =dense_depth_map(px,n, m,4);
    toc
    %保存对应的深度图像为uint16格式，单位为毫米。
    imwrite(uint16(depth*1000),[depth_map_save_path,file_id,'.png']);
% 
%     figure;imagesc(fulldepth,[0 30]);
%     axis image
%     axis off
%     title('Full Depth map grid=4');
    
%      figure;imagesc(depth);
%      axis image
%      axis off
%      title('Initial Depth map');


    % Composite image of grayscale left image and disparity map 

%     tmp(:,:,1) = double(rgb2gray(I))/255;
%     tmp(:,:,2) =tmp(:,:,1);
%     tmp(:,:,3) =tmp(:,:,1);
% 
%     dmap=1./fulldepth;
%     dmap(isinf(dmap))=0;
%     dmap=63*(dmap-min(dmap(:)))./(max(dmap(:))-min(dmap(:)));
%     dmap=round(dmap);


 %   figure;sc= colormap('jet');
 %   DImage = 0.5*tmp + 0.5*reshape(sc(dmap+1,:), [n,m ,3]);
%     imshow((DImage));title('Inverse of Depth');
end

