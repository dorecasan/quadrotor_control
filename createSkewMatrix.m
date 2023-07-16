function skewMatrix = createSkewMatrix(vector)
    % Check if the input is a 3D vector
    if numel(vector) ~= 3
        error('Input vector must have 3 elements');
    end
    
    % Extract the vector components
    v1 = vector(1);
    v2 = vector(2);
    v3 = vector(3);

    % Create the skew matrix
    skewMatrix = [0, -v3, v2;
                  v3, 0, -v1;
                  -v2, v1, 0];
end
