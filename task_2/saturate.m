function [value] = saturate(value, lower_limit, upper_limit)

if (value < lower_limit)
    value = lower_limit;
elseif (value > upper_limit)
    value = upper_limit;
end

end