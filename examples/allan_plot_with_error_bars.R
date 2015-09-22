d=data.frame(drink=c(1,100,10000), mean=c(3,6,2), sd=c(2.6,5.6,1.8))

library("ggplot2")
m = ggplot(data=d, aes(x=drink, y=mean, group=1))

geom_line() + geom_point()+
geom_errorbar(mapping=aes(x=drink, ymin=mean-sd, ymax=mean+sd), width=0.2, size=1, color="blue", scale="free")+
scale_x_log10() + scale_y_log10()
#geom_point(data=d, mapping=aes(x=drink, y=mean), size=4, shape=21, fill="white")+



##install.packages("Hmisc", dependencies=T)
library("Hmisc")

# add error bars (without adjusting yrange)
plot(d$drink, d$mean, type="n")
with (
  data = d
  , expr = errbar(drink, mean, mean+sd, mean-sd, add=F, pch=1, cap=.015, log="xy")
)


d = data.frame(
  x  = c(1:5)
  , y  = c(1.1, 1.5, 2.9, 3.8, 5.2)
  , sd = c(0.2, 0.3, 0.2, 0.0, 0.4)
)

plot (d$x, d$y, ylim=c(0, 6), log="xy")
epsilon = 0.02

segments(d$x, d$y-d$sd,d$x, d$y+d$sd)
epsilon = 0.02
segments(d$x-epsilon,d$y-d$sd,d$x+epsilon,d$y-d$sd)
segments(d$x-epsilon,d$y+d$sd,d$x+epsilon,d$y+d$sd)




###################################
# Combine two different axes scales
#

library(ggplot2)
library(gtable)
library(grid)

d=data.frame(drink=c(1,100,10000), mean=c(3,6,2), sd=c(2.6,5.6,1.8))

p1 <- ggplot(data=d, aes(x=drink, y=mean, group=1)) + geom_line(aes(x=drink, y=mean)) + geom_point() + scale_x_log10() + scale_y_log10()
p2 <- ggplot()+geom_errorbar(data=d, mapping=aes(x=drink, ymin=mean-sd, ymax=mean+sd), width=0.2, size=1, color="blue", scale="free")


#extract gtable
g1<-ggplot_gtable(ggplot_build(p1))
g2<-ggplot_gtable(ggplot_build(p2))

#overlap the panel of the 2nd plot on that of the 1st plot

pp<-c(subset(g1$layout, name=="panel", se=t:r))
g<-gtable_add_grob(g1, g2$grobs[[which(g2$layout$name=="panel")]], pp$t, pp$l, pp$b, pp$l)

ia <- which(g2$layout$name == "axis-l")
ga <- g2$grobs[[ia]]
ax <- ga$children[[2]]
ax$widths <- rev(ax$widths)
ax$grobs <- rev(ax$grobs)
ax$grobs[[1]]$x <- ax$grobs[[1]]$x - unit(1, "npc") + unit(0.15, "cm")
g <- gtable_add_cols(g, g2$widths[g2$layout[ia, ]$l], length(g$widths) - 1)
g <- gtable_add_grob(g, ax, pp$t, length(g$widths) - 1, pp$b)

# draw it
grid.draw(g)


