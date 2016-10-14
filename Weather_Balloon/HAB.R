library(ggplot2)
library(plyr)
library(scales)
library(directlabels)
library(reshape2)
setwd("/home/peanut/Arduino/Weather_Balloon/")
HABdata <- read.csv("./HAB_data_2_edited.csv", header=T, na.strings=-1,row.names=NULL)

HABdata$flighttime <- HABdata[["unix_time"]] - min(HABdata$unix_time)
fancy_scientific <- function (l) 
{
    l <- format(l, scientific = TRUE, nsmall=3, digits=3)
    l <- gsub("^(.*)e", "'\\1'e", l)
    l <- gsub("e", "%*%10^", l)
    parse(text = l)
}

myTheme <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=20), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black", angle=45, hjust=1), axis.title.y=element_text(vjust=1))
myThemeNoAngle <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=20), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black"), axis.title.y=element_text(vjust=1))
myThemeNoAngleSmaller <- theme(panel.grid.major = element_line(color="darkgrey", size=0.5), panel.grid.minor = element_line(color="darkgrey", size=0.5, linetype="dashed"), text = element_text(size=15), plot.title = element_text(vjust=1), panel.background = element_rect(fill="white"), axis.text.y = element_text(color="black"), axis.text.x = element_text(color="black"), axis.title.y=element_text(vjust=1))

# Replace for black background
#myTheme <- theme(axis.ticks = element_line(colour="white"), line=element_line(colour="white"), panel.grid.major = element_line(colour="blue", size=0.5), panel.grid.minor = element_line(colour="blue", size=0.5, linetype="dashed"), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x = element_text(angle=45, hjust=1, colour="white"))
#myThemeNoAngle <- theme(axis.ticks = element_line(colour="white"), line = element_line(colour="white",panel.grid.major = element_line(colour="blue", size=0.5), panel.grid.minor = element_line(colour="blue", size=0.5, linetype="dashed"), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x=element_text(colour="white"))
#myTheme <- theme(axis.ticks = element_line(colour="white"), line=element_line(colour="white"), panel.grid.major = element_line(colour="#414141", size=0.5, linetype="dashed"), panel.grid.minor = element_line(colour=NA), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x = element_text(angle=45, hjust=1, colour="white"))
#myThemeNoAngle <- theme(axis.ticks = element_line(colour="white"), line = element_line(colour="white"), panel.grid.major = element_line(colour="#414141", size=0.5, linetype="dashed"), panel.grid.minor = element_line(colour=NA), text = element_text(size=24, colour="white"), plot.title = element_text(vjust=1), panel.background = element_blank(), plot.background = element_blank(), axis.text.y = element_text(vjust=1, colour="white"), axis.text.x=element_text(colour="white"))

#sortList <- rename(sortList, c("QuickBook" = "Quick - Book", "QuickInternet1"="Quick - Internet"))
#sortFractionList <- subset(sortList, select=c("ListSize", "Merge", "Quick - Book", "Quick - Internet"))
 
altitude <- ggplot(HABdata, aes(x=flighttime, y=altitude)) + geom_line(size=1) + myTheme + xlab("Unix Time (s)") + ylab("GPS Altitude (m)")
altitude
ggsave("altitude_time.png", width=12.8, height=8, dpi=600, bg="transparent")

radiation <- ggplot(HABdata, aes(x=flighttime, y=geiger_x)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Unix Time (s)") + ylab("Instantaneous radiation count (event)")
radiation
ggsave("rad_time.png", width=12.8, height=8, dpi=600, bg="transparent")

radiation <- ggplot(HABdata, aes(x=altitude, y=geiger_x)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("Instantaneous radiation count (event)")
radiation
ggsave("rad_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

speed <- ggplot(HABdata, aes(x=flighttime, y=speed)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Speed (m/s)")
speed
ggsave("speed_time.png", width=12.8, height=8, dpi=600, bg="transparent")

speed <- ggplot(HABdata, aes(x=altitude, y=speed)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("Speed (m/s)")
speed
ggsave("speed_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_external <- subset(HABdata, temp_external != -1000 & temp_external != -127)

external <- ggplot(HABdata_external, aes(x=altitude, y=temp_external)) + geom_point() + myTheme + xlab("Altitude (m)") + ylab("External Temp (C)")
external
ggsave("external_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

external <- ggplot(HABdata_external, aes(x=flighttime, y=temp_external)) + geom_point() + myTheme + xlab("Flight time (s)") + ylab("External Temp (C)")
external
ggsave("external_time.png", width=12.8, height=8, dpi=600, bg="transparent")

HABdata_internal <- subset(HABdata, temp_internal != -1000)

internal <- ggplot(HABdata_internal, aes(x=altitude, y=temp_internal)) + geom_point() + myTheme + xlab("Altitude (m)") + ylab("Internal Temp (C)")
internal
ggsave("internal_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

internal <- ggplot(HABdata_internal, aes(x=flighttime, y=temp_internal)) + geom_point() + myTheme + xlab("Flight time (s)") + ylab("Internal Temp (C)")
internal
ggsave("internal_time.png", width=12.8, height=8, dpi=600, bg="transparent")

pressure <- ggplot(HABdata, aes(x=altitude, y=pressure)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("Pressure (Pa)")
pressure
ggsave("pressure_altitude.png", width=12.8, height=8, dpi=600, bg="transparent")

pressure <- ggplot(HABdata, aes(x=flighttime, y=pressure)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Pressure (Pa)")
pressure
ggsave("pressure_time.png", width=12.8, height=8, dpi=600, bg="transparent")

uv <- ggplot(HABdata, aes(x=altitude, y=uv)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Altitude (m)") + ylab("UV (analog)")
uv
ggsave("uv_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

course <- ggplot(HABdata, aes(x=flighttime, y=course)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("GPS Course (Degrees)")
course
ggsave("GPS_course_time.png", width=12.8, height=8, dpi=600, bg="transparent")

humidity <- ggplot(HABdata, aes(x=altitude, y=humidity)) + geom_point() + myTheme + xlab("Altitude (m)") + ylab("Uncompensated RH (%)")
humidity
ggsave("humidity_alt.png", width=12.8, height=8, dpi=600, bg="transparent")

humidity <- ggplot(HABdata, aes(x=flighttime, y=humidity)) + geom_point() + myTheme + xlab("Flight time (s)") + ylab("Uncompensated RH (%)")
humidity
ggsave("humidity_time.png", width=12.8, height=8, dpi=600, bg="transparent")

voltage <- ggplot(HABdata, aes(x=flighttime, y=voltage)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Battery Voltage")
voltage
ggsave("voltage_time.png", width=12.8, height=8, dpi=600, bg="transparent")

voltage <- ggplot(HABdata, aes(x=flighttime, y=state_of_charge)) + geom_point() + geom_smooth(color="red") + myTheme + xlab("Flight time (s)") + ylab("Battery State of Charge (% remaining)")
voltage
ggsave("soc_time.png", width=12.8, height=8, dpi=600, bg="transparent")
