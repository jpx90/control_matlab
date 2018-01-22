function y = control_curve(x)

if x < -0.2
    y = x - 0.15;
elseif x < -0.1
    y = x * x * 5 + x * 3 + 0.05;
elseif x < 0.1
    y = x * 2;
elseif x < 0.2
    y = -x * x * 5 + x * 3 - 0.05;
else
    y = x + 0.15;
end

end