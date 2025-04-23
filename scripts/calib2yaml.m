% This MATLAB function is used to save the calibration parameters to a yaml file

function calib2yaml(cameraParams, filePath)
    % Create the yaml file
    fileID = fopen(filePath, 'w');

    % Write the image size
    fprintf(fileID, 'width: %d\n', cameraParams.ImageSize(2));
    fprintf(fileID, 'height: %d\n', cameraParams.ImageSize(1));

    % Write the distortion coefficients
    distortion = [cameraParams.RadialDistortion(1:2), cameraParams.TangentialDistortion, cameraParams.RadialDistortion(3)];
    fprintf(fileID, 'dist:\n');
    for i = 1:size(distortion, 1)
        for j = 1:size(distortion, 2)
            if j == 1
                fprintf(fileID, '  - - %f\n', distortion(i, j));
            else
                fprintf(fileID, '    - %f\n', distortion(i, j));
            end
        end
    end

    % Write the camera matrix
    fprintf(fileID, 'mtx:\n');
    for i = 1:size(cameraParams.IntrinsicMatrix, 1)
        for j = 1:size(cameraParams.IntrinsicMatrix, 2)
            if j == 1
                fprintf(fileID, '  - - %f\n', cameraParams.IntrinsicMatrix(j, i));
            else
                fprintf(fileID, '    - %f\n', cameraParams.IntrinsicMatrix(j, i));
            end
        end
    end
end

