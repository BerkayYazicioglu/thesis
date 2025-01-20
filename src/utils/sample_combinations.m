function output = sample_combinations(array, k, numSamples)
    % array: Input array to sample from
    % k: Size of each combination
    % numSamples: Number of combinations to sample
    % Returns: matrix of sampled combinations
    
    function binom = nchoosekFast(n_, k_)
        % Efficiently computes binomial coefficient without generating combinations
        if k_ > n_ || k_ < 0
            binom = 0;
        elseif k_ == 0 || k_ == n_
            binom = 1;
        else
            k_ = min(k_, n_ - k_); % Take advantage of symmetry
            binom = prod((n_ - k_ + 1):n_) / prod(1:k_);
        end
    end

    n = length(array);
    totalCombinations = factorial(n) / (factorial(k) * factorial(n - k)); 
    if isnan(totalCombinations)
        error('Number of elements in the input array is too large');
    end
    
    if totalCombinations <= numSamples
        output = nchoosek(array, k);
        return;
    end
    sampleIndices = round(linspace(0, totalCombinations - 1, numSamples));
    output = [];
 
    % Convert indices to combinations
    for idx = 1:numSamples
        m = sampleIndices(idx);
        out = zeros(1,n);
        r = k;
        n_ = n;
        while (n_ > 0)
            if n_ > r && r >= 0
                y = nchoosekFast(n_-1, r);
            else
                y = 0;
            end
            if m >= y
                m = m - y;
                out(n_) = 1;
                r = r - 1;
            else
                out(n_) = 0;
            end
            n_ = n_ - 1;
        end
        if sum(out) ~= k
            continue
        end
        output(end+1, :) = array(logical(out));
    end
    if isempty(output)
        error("something is wrong")
    end
end

