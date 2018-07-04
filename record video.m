Record Video:writerObj = VideoWriter('title');
open(witerObj);
frame = getframe(gcf);
writeVideo(writerObj,frame);
close(writerObj);
