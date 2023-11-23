function angle = computeAngle(course)

if (0 <= course) && (course <=90)
    angle = 90-course;
else
    angle = 450-course;
end

end