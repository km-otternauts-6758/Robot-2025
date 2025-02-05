import wpilib
from networktables import NetworkTables
import robotpy_apriltag


class LimeLight:
    def robotInit(self):
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getTable("limelight-kmrobot")
        print(f"tv: {self.limelight.getEntry("tv")}")


    # def lighting(self):
    #     # print(f"ta: {self.limelight.getEntry("ta")}")
    #     # print(f"tv: {self.limelight.getEntry("tv")}")
    #     # print(self.limelight.getValue("ta", 0))
    #     if self.limelight.getNumber("ta", 0) > 1:
    #         self.limelight.putNumber("ledMode", 1)
    #     else:
    #         self.limelight.putNumber("ledMode", 3)

    def visible(self, key:str, defaultValue:float):
       return self.limelight.getNumber(key, defaultValue)

#         # self.tkugglechuzzler = tkinter + bugglefug + huzz + chud + rizzler (https://i5.walmartimages.com/seo/GZSL-Fuggler-Plush-Toys-Fuggler-Funny-Ugly-Monster-Glow-in-the-Dark-Limited-Edition-Funny-Holiday-Gift-fo-Kids-Age-8-Brown-Grin-Grin-One-Size_c4e3cea2-bc65-457f-a88c-f3b512935fba.a97979d39e54a1151ac5c2b9c7ca8928.jpeg)
