function noisy_data = GaussianNoiseGenerator(data, sigma)
%GAUSSIANNOISEGENERATOR Adds Gaussian noise to input data
%   noisy_data = GuassianNoiseGenerator(data, sigma) returns the input 
%   data with added Gaussian noise.
%
%   Inputs:
%   - data: The original data (can be a vector or matrix).
%   - sigma: The standard deviation of the Gaussian noise.
%
%   Output:
%   - noisy_data: Data with added Gaussian noise.

    % Generate Gaussian noise
    noise = sigma * randn(size(data));
    
    % Add the noise to the input data
    noisy_data = data + noise;
end

