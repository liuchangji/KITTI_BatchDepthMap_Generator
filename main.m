% Written by Muhammet Balcilar, France
% All rights reserved
% LCJ�޸ģ���������ϡ�����ͼ��
% Դ���������ڲ��ͣ�https://blog.csdn.net/linghugoolge/article/details/85222527
% �ļ��нṹ��
%--Data---|
%         |- calib (�궨�����ļ���
%         |- image_2 (��Ŀͼ��
%         |- velodyne (�����״
%         |- depth (�ֶ��½�һ�����depthͼ���ļ��У�
%
% data_path,���иó���
clear all
clc;
data_path =  'C:/liuchangji/����ʵ���ָ��Ŀ����άλ�ù��Ʒ���/KITTI/object/training/';
image_2_path = [data_path,'image_2/'];
calib_path = [data_path,'calib/'];
bin_path = [data_path,'velodyne/'];
depth_map_save_path = [data_path,'depth/'];
File = dir(fullfile(image_2_path,'*.png'));  % ��ʾ�ļ��������з��Ϻ�׺��Ϊ.png�ļ���������Ϣ
FileNames = {File.name}';                    % ��ȡ���Ϻ�׺��Ϊ.txt�������ļ����ļ�����ת��Ϊn��1��
FileNumbers = size(FileNames);
FileNumbers = FileNumbers(1);

for i=1:FileNumbers
    process_information = fprintf('���ڴ���%d/%d    ',i,FileNumbers);
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
    %�����Ӧ�����ͼ��Ϊuint16��ʽ����λΪ���ס�
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

