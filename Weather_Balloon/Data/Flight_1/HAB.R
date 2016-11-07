library(ggplot2)
library(plyr)
library(scales)
library(directlabels)
library(reshape2)
setwd("/home/peanut/Arduino/Weather_Balloon/")
HABdata <- read.csv("./HAB_data_2_edited.csv", header=T, na.strings=-1,row.names=NULL)
HABdata_early <- read.csv("./HAB_data_2_edit_less.csv", header=T, na.strings=-1,row.names=NULL)

HABdata$flighttime <- HABdata[["unix_time"]] - min(HABdata$unix_time)
HABdata_early$flighttime <- HABdata_early[["unix_time"]] - min(HABdata_early$unix_time)

fancy_scientific <- function (l) 
{
    l <- format(l, scientific = TRUE, nsmall=3, digits=3)
    l <- gsub("^(.*)e", "'\\1'e", l)
    l <- gsub("e", "%*%10^", l)
    parse(text = l)
}

myTheme <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=20), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black", angle=45, hjust=1), axis.title.y=element_text(vjust=1))
myThemeNoAngle <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=20), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black"), axis.title.y=element_text(vjust=1))
myThemeNoAngleSmaller <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=1), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black"), axis.title.y=element_text(vjust=1))

# Replace for black background
#myTheme <- theme(axis.ticks = element_line(colour="white"), line=element_line(colour="white"), panel.grid.major = element_line(colour="blue", size=0.5), panel.grid.minor = element_line(colour="blue", size=0.5, linetype="dashed"), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x = element_text(angle=45, hjust=1, colour="white"))
#myThemeNoAngle <- theme(axis.ticks = element_line(colour="white"), line = element_line(colour="white",panel.grid.major = element_line(colour="blue", size=0.5), panel.grid.minor = element_line(colour="blue", size=0.5, linetype="dashed"), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x=element_text(colour="white"))
#myTheme <- theme(axis.ticks = element_line(colour="white"), line=element_line(colour="white"), panel.grid.major = element_line(colour="#414141", size=0.5, linetype="dashed"), panel.grid.minor = element_line(colour=NA), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x = element_text(angle=45, hjust=1, colour="white"))
#myThemeNoAngle <- theme(axis.ticks = element_line(colour="white"), line = element_line(colour="white"), panel.grid.major = element_line(colour="#414141", size=0.5, linetype="dashed"), panel.grid.minor = element_line(colour=NA), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x=element_text(colour="white"))

#sortList <- rename(sortList, c("QuickBook" = "Quick - Book", "QuickInternet1"="Quick - Internet"))
#sortFractionList <- subset(sortList, select=c("ListSize", "Merge", "Quick - Book", "Quick - Internet"))
 
altitude <- ggplot(HABdata, aes(x=flighttime, y=altitude)) + geom_line(color="red", size=0.75) + geom_line(size=1) + myTheme + xlab("Flight time (s)") + ylab("GPS Altitude (m)")
altitude
ggsave("altitude_time.png", width=12.8, height=8, dpi=600, bg="transparent")

radiation <- ggplot(HABdata, aes(x=flighttime, y=geiger_x)) + geom_point(size=1) + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Instantaneous radiation count (event)")
radiation
ggsave("rad_time.png", width=12.8, height=8, dpi=600, bg="transparent")

radiation <- ggplot(HABdata, aes(x=altitude, y=geiger_x)) + geom_point(size=1) + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("Instantaneous radiation count (event)")
radiation
ggsave("rad_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

speed <- ggplot(HABdata, aes(x=flighttime, y=speed)) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Speed (m/s)")
speed
ggsave("speed_time.png", width=12.8, height=8, dpi=600, bg="transparent")

speed <- ggplot(HABdata, aes(x=altitude, y=speed)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("Speed (m/s)")
speed
ggsave("speed_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_external <- subset(HABdata, temp_external != -1000 & temp_external != -127)

external <- ggplot(HABdata_external, aes(x=altitude, y=temp_external)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("External Temp (C)")
external
ggsave("external_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

external <- ggplot(HABdata_external, aes(x=flighttime, y=temp_external)) + geom_line(color="red") + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("External Temp (C)")
external
ggsave("external_time.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_internal <- subset(HABdata, temp_internal != -1000)

internal <- ggplot(HABdata_internal, aes(x=altitude, y=temp_internal)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("Internal Temp (C)")
internal
ggsave("internal_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

internal <- ggplot(HABdata_internal, aes(x=flighttime, y=temp_internal)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Internal Temp (C)")
internal
ggsave("internal_time.png", width=12.8, height=8, dpi=600, bg="transparent")

main <- ggplot(HABdata, aes(x=altitude, y=temp_main)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("Control Board Temp (C)")
main
ggsave("main_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

main <- ggplot(HABdata, aes(x=flighttime, y=temp_main)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Control Board Temp (C)")
main
ggsave("main_time.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_tsl <- subset(HABdata, temp_tsl2561 != -1000)
tsl <- ggplot(HABdata_tsl, aes(x=flighttime, y=temp_tsl2561)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("TSL2561 Temp (C)")
tsl
ggsave("tsl2561_time.png", width=12.8, height=8, dpi=600, bg="transparent")

tsl <- ggplot(HABdata_tsl, aes(x=altitude, y=temp_tsl2561)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("TSL2561 Temp (C)")
tsl
ggsave("tsl2561_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

pressure <- ggplot(HABdata, aes(x=altitude, y=pressure)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("Pressure (Pa)")
pressure
ggsave("pressure_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

pressure <- ggplot(HABdata, aes(x=flighttime, y=pressure)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Pressure (Pa)")
pressure
ggsave("pressure_time.png", width=12.8, height=8, dpi=600, bg="transparent")

uv <- ggplot(HABdata, aes(x=flighttime, y=uv)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("UV (analog)")
uv
ggsave("uv_time.png", width=12.8, height=8, dpi=600, bg="transparent")

uv <- ggplot(HABdata, aes(x=altitude, y=uv)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("UV (analog)")
uv
ggsave("uv_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

course <- ggplot(HABdata, aes(x=flighttime, y=course)) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("GPS Course (Degrees)")
course
ggsave("GPS_course_time.png", width=12.8, height=8, dpi=600, bg="transparent")

humidity <- ggplot(HABdata, aes(x=altitude, y=humidity)) + geom_point(size=1) + myTheme + xlab("Altitude (m)") + ylab("Uncompensated RH (%)")
humidity
ggsave("humidity_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

humidity <- ggplot(HABdata, aes(x=flighttime, y=humidity)) + geom_line(color="red", size=0.75) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Uncompensated RH (%)")
humidity
ggsave("humidity_time.png", width=12.8, height=8, dpi=600, bg="transparent")

voltage <- ggplot(HABdata, aes(x=flighttime, y=voltage)) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Battery Voltage")
voltage
ggsave("voltage_time.png", width=12.8, height=8, dpi=600, bg="transparent")

voltage <- ggplot(HABdata, aes(x=flighttime, y=state_of_charge)) + geom_point(size=1) + myTheme + xlab("Flight time (s)") + ylab("Battery State of Charge (% remaining)")
voltage
ggsave("soc_time.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_early$accel_mag <- sqrt(HABdata_early[["accel_x"]]^2 + HABdata_early[["accel_y"]]^2 + HABdata_early[["accel_z"]]^2)
mean(HABdata_early[430:970,]$accel_mag)
HABdata$accel_x <- HABdata$accel_x / (mean(HABdata_early[430:970,]$accel_mag))
HABdata$accel_y <- HABdata$accel_y / (mean(HABdata_early[430:970,]$accel_mag))
HABdata$accel_z <- HABdata$accel_z / (mean(HABdata_early[430:970,]$accel_mag))
HABdata$gyro_x <- HABdata$gyro_x - mean(HABdata_early[430:970,]$gyro_x)
HABdata$gyro_y <- HABdata$gyro_y - mean(HABdata_early[430:970,]$gyro_y)
HABdata$gyro_z <- HABdata$gyro_z - mean(HABdata_early[430:970,]$gyro_z)

HABdata$accel_mag <- sqrt(HABdata[["accel_x"]]^2 + HABdata[["accel_y"]]^2 + HABdata[["accel_z"]]^2)

write.csv(HABdata, file = "HAB_corrected.csv", row.names = FALSE)

accel <- ggplot(HABdata, aes(x=flighttime, y=accel_mag)) + geom_point(size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("accel_mag_time.png", width=12.8, height=8, dpi=600, bg="transparent")

accel <- ggplot(HABdata, aes(x=altitude, y=accel_mag)) + geom_point(size=0.75) + myTheme + xlab("Altitude (m)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("accel_mag_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata, select=c("flighttime", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="flighttime")

accel <- ggplot(accelLong, aes(x=flighttime, y=value)) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Acceleration (g)") + theme(legend.position = c(0.2,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("accel_time.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata, select=c("altitude", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="altitude")

accel <- ggplot(accelLong, aes(x=altitude, y=value)) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Altitude (m)") + ylab("Acceleration (g)") + theme(legend.position = c(0.2,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("accel_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

gyro.df <- subset(HABdata, select=c("flighttime", "gyro_x", "gyro_y", "gyro_z"))#, "accel_mag"))
gyro.df <- rename(gyro.df, c("gyro_x" = "X", "gyro_y"="Y", "gyro_z"="Z"))#, "accel_mag"="Mag"))
gyroLong <- melt(gyro.df, id.vars="flighttime")
gyro <- ggplot(gyroLong, aes(x=flighttime, y=value)) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Gyro (degrees/sec)") + theme(legend.position = c(0.2,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
gyro
ggsave("gyro_time.png", width=12.8, height=8, dpi=600, bg="transparent")

# Grab decent data
HABdata_fall <- subset(HABdata, flighttime > 12500)

accel <- ggplot(HABdata_fall, aes(x=flighttime, y=accel_mag)) + geom_point(size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("decent_accel_mag_time.png", width=12.8, height=8, dpi=600, bg="transparent")

accel <- ggplot(HABdata_fall, aes(x=altitude, y=accel_mag)) + geom_point(size=0.75) + myTheme + xlab("Altitude (m)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("decent_accel_mag_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata_fall, select=c("flighttime", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="flighttime")

accel <- ggplot(accelLong, aes(x=flighttime, y=value)) + geom_point(aes(color=variable), size=0.75)  + myTheme + xlab("Flight time (s)") + ylab("Acceleration (g)") + theme(legend.position = c(0.1,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("decent_accel_time.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata_fall, select=c("altitude", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="altitude")

accel <- ggplot(accelLong, aes(x=altitude, y=value)) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Altitude (m)") + ylab("Acceleration (g)") + theme(legend.position = c(0.1,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("decent_accel_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

gyro.df <- subset(HABdata_fall, select=c("flighttime", "gyro_x", "gyro_y", "gyro_z"))#, "accel_mag"))
gyro.df <- rename(gyro.df, c("gyro_x" = "X", "gyro_y"="Y", "gyro_z"="Z"))#, "accel_mag"="Mag"))
gyroLong <- melt(gyro.df, id.vars="flighttime")
gyro <- ggplot(gyroLong, aes(x=flighttime, y=value)) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Gyro (degrees/sec)") + theme(legend.position = c(0.1,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
gyro
ggsave("decent_gyro_time.png", width=12.8, height=8, dpi=600, bg="transparent")

# Grab 10 minutes of data
HABdata_one_min <- subset(HABdata, flighttime >= 5000 & flighttime <= 5600)
accel <- ggplot(HABdata_one_min, aes(x=flighttime, y=accel_mag)) + geom_point(size=0.75) + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("accel_mag_time_10_min.png", width=12.8, height=8, dpi=600, bg="transparent")

accel <- ggplot(HABdata_one_min, aes(x=altitude, y=accel_mag)) + geom_point(size=0.75) + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("Acceleration Magnitude (g)")
accel
ggsave("accel_mag_altitude_10_min.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata_one_min, select=c("flighttime", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="flighttime")

accel <- ggplot(accelLong, aes(x=flighttime, y=value)) + geom_line(aes(color=variable), size=0.75) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Acceleration (g)") + theme(legend.position = c(0.9,0.8)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("accel_time_10_min.png", width=12.8, height=8, dpi=600, bg="transparent")

accel.df <- subset(HABdata_one_min, select=c("altitude", "accel_x", "accel_y", "accel_z"))#, "accel_mag"))
accel.df <- rename(accel.df, c("accel_x" = "X", "accel_y"="Y", "accel_z"="Z"))#, "accel_mag"="Mag"))
accelLong <- melt(accel.df, id.vars="altitude")

accel <- ggplot(accelLong, aes(x=altitude, y=value)) + geom_line(aes(color=variable), size=0.75) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Altitude (m)") + ylab("Acceleration (g)") + theme(legend.position = c(0.9,0.8)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
accel
ggsave("accel_alt_10_min.png", width=12.8, height=8, dpi=600, bg="transparent")

gyro.df <- subset(HABdata_one_min, select=c("flighttime", "gyro_x", "gyro_y", "gyro_z"))#, "accel_mag"))
gyro.df <- rename(gyro.df, c("gyro_x" = "X", "gyro_y"="Y", "gyro_z"="Z"))#, "accel_mag"="Mag"))
gyroLong <- melt(gyro.df, id.vars="flighttime")
gyro <- ggplot(gyroLong, aes(x=flighttime, y=value)) + geom_line(aes(color=variable), size=0.75) + geom_point(aes(color=variable), size=0.75) + myTheme + xlab("Flight time (s)") + ylab("Gyro (degrees/sec)") + theme(legend.position = c(0.1,0.9)) + guides(color=guide_legend(title="Axis"))#+ scale_y_continuous(labels=fancy_scientific)
gyro
ggsave("gyro_time_10_min.png", width=12.8, height=8, dpi=600, bg="transparent")