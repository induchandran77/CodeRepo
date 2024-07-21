import numpy as np


class point: 
    def __init__(self,W,L):
        self.L = L 
        self.W = W
        self.w = 0
        self.assignedList= []
        self.boundedList = []
        self.point1 = None
        self.point2 = None
        self.hmin = 50
        self.width = 0
        self.valid = True

    def calc_dimensions(self, h, alpha):
        aspect_ratio = (16/9)
        Lx = 2 * h * np.tan(alpha / 2)
        Ly = Lx / aspect_ratio
        return Lx, Ly

    def form_tuple(self, Lx, Ly, h):
        self.coord_1 = (Lx / 2)
        self.coord_2 = Ly / 2
        return (self.coord_1, self.coord_2, h)

    def tuple_adjust(self,t):
        t_list = list(t)
        index_to_modify = 0
        t_list[index_to_modify] += self.width
        new_tuple = tuple(t_list)
        return new_tuple

    def appending_to_list(self, point, range, alpha):
        self.assignedList.append([point, range, alpha])
        
        return

    def calc_distance(self, point1, point2):
        distance = np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
        return distance

    def altitude_adjust(self, pointA,rangeA, alphaA,pointB,rangeB,alphaB): #accepts tuple of positions, range,alpha
        # Assuming point1 > point2
        hmin = self.hmin
        h1 = pointA[2]
        r1 = rangeA
        alpha1 = alphaA

        h2 = pointB[2]
        r2 = rangeB
        alpha2 = alphaB
        modified_pointA = None
        modified_pointB = None
        print("h1,hmin",h1,hmin)

        while (h1 - hmin) > 0.1:
            h = (hmin + h1) / 2 
            print("h_new",h) 
            LxA,LyA = self.calc_dimensions(h, alpha1)
            print("LxA,LyA",LxA,LyA)
            modified_pointA = self.form_tuple(LxA, LyA, h) #here 
            
            modified_pointA = self.tuple_adjust(self.form_tuple(LxA, LyA, h))
            print("modified_pointA, pointB,1",modified_pointA, pointB)
            LxB,LyB = self.calc_dimensions(h2, alpha2)
            print("LxB,LyB",LxB,LyB)
            pointB = self.form_tuple(LxB,LyB,h2)
            B_list = list(pointB)
            index_to_modify = 0
            B_list[index_to_modify] += LxA
            modified_pointB = tuple(B_list)

            print("modified_pointA, modified_pointB,2",modified_pointA, modified_pointB)
            dist = self.calc_distance(modified_pointA, modified_pointB)
            print("mod_dist",dist)
            if dist >= min(self.r1, self.r2):
                h1 = h
            else:
                hmin = h
        
        dist = self.calc_distance(modified_pointA, pointB)    
        if dist >= min(self.r1, self.r2):
            print("not valid result")  
            self.valid = False  
        else:
            self.valid = True     
        print("modified_pointA,modified_pointB",modified_pointA,modified_pointB)
        return modified_pointA,modified_pointB,LxA,LxB
    
    def altitude_adjust_2(self, pointA,rangeA, alphaA,pointB,rangeB,alphaB): #accepts tuple of positions, range,alpha
        # Assuming point1 > point2
        print("pointA,rangeA, alphaA,pointB,rangeB,alphaB",pointA,rangeA, alphaA,pointB,rangeB,alphaB)
        hmin = self.hmin
        h1 = pointA[2]
        r1 = rangeA
        alpha1 = alphaA

        h2 = pointB[2]
        r2 = rangeB
        alpha2 = alphaB
        
        while (h1 - hmin) > 0.1:

                h = (hmin + h1) / 2 
                print("h_mod",h) 
                LxA,LyA = self.calc_dimensions(h, alpha1)
                modified_pointA = self.form_tuple(LxA, LyA, h) #here 
                print("modified_pointA, pointB,1",modified_pointA, pointB)
                modified_pointA = self.tuple_adjust(modified_pointA)
                
                #LxB,LyB = self.calc_dimensions(h2, alpha2)
                #pointB = self.form_tuple(LxB,LyB,h2)
                #B_list = list(pointB)
                #index_to_modify = 0
                #B_list[index_to_modify] += LxA
                #modified_pointB = tuple(B_list)

                print("modified_pointA, modified_pointB,2",modified_pointA)
                dist = self.calc_distance(modified_pointA, pointB)
                print("mod_dist",dist)
                if dist >= min(self.r1, self.r2):
                    h1 = h
                else:
                    hmin = h
                print("h1,hmin",h1,hmin)
        dist = self.calc_distance(modified_pointA, pointB)    
        if dist >= min(self.r1, self.r2):
            print("not valid result")  
            self.valid = False  
        else:
            self.valid = True    
        print("modified_pointA&B",modified_pointA) 
        return modified_pointA
    
    def placing(self, combination):
        print("flag1", combination)
        for i, point_values in enumerate(combination):
            if i == 0:
                self.h1, self.r1, self.alpha1 = point_values
                print("self.alpha1", self.alpha1)
                Lx, Ly = self.calc_dimensions(self.h1, self.alpha1)
                print("Lx", Lx) #Lx 497.9566889019861
                print("Ly", Ly) #Ly 280.1006375073672
                self.point_1 = self.form_tuple(Lx, Ly, self.h1)  #(248.97834445099306, 140.0503187536836, 230)
                print("self.point1", self.point_1)
                self.point1 = self.tuple_adjust(self.point_1)  #(248.97834445099306, 140.0503187536836, 230)
                print("tuple adjust", self.point1)
                self.appending_to_list(self.point1, self.r1, self.alpha1)
                self.width += Lx
            else:
                print(i)
                print("self.assignedList_updates",self.assignedList)
                modified_list = []
                popped_value = self.assignedList.pop() # [[(248.97834445099306, 140.0503187536836, 230), 250, 1.65]]
                print("value popped", popped_value)
                self.point1, self.r1, self.alpha1 = popped_value
                
                self.h2, self.r2, self.alpha2 = point_values

                Lx, Ly = self.calc_dimensions(self.h2, self.alpha2) 
                self.point_2 = self.form_tuple(Lx, Ly, self.h2) 
                print("self.point2", self.point_2)  #(270.62863527281854, 152.22860734096045, 250)
                self.point2 = self.tuple_adjust(self.point_2) #(768.5853241748047, 152.22860734096045, 250)
                print("self.point2 tupled", self.point2)
                dist = self.calc_distance(self.point1, self.point2) 
                print("distance",dist) #520.1343327358967
                if dist > min(self.r1, self.r2):#yes 520>250
                    print("flag IN")
                    if self.h1 > self.h2: # No
                        Lx, Ly = self.calc_dimensions(self.h1, self.alpha1) 
                        self.width -=Lx
                        print("flagh1>h")
                        print("self.point1,self.r1,self.alpha1,self.point2,self.r2,self.alpha2",self.point1,self.r1,self.alpha1,self.point2,self.r2,self.alpha2)
                        self.point1,self.point2,Lx1,Lx2 = self.altitude_adjust(self.point1,self.r1,self.alpha1,self.point2,self.r2,self.alpha2)
                        if self.valid == True:   
                            self.h1 = self.point1[2]
                            self.width += (Lx1+Lx2)
                            modified_list.append([self.point1, self.r1, self.alpha1])
                            modified_list.append((self.point2, self.r2, self.alpha2))

                            while self.assignedList:
                                popped_value = self.assignedList.pop()
                                self.point2, self.r2, self.alpha2 = popped_value
                        
                                dist = self.calc_distance(self.point1, self.point2)
                                if dist > min(self.r1, self.r2):
                                    if self.point2[2] > self.h1:
                                        self.point2,self.point1,Lx,Ly = self.altitude_adjust(self.point2, self.r2,self.alpha2, self.point1,self.r1,self.alpha1)
                                        if self.valid ==True:
                                            self.point1 = self.point2
                                            self.r1 = self.r2
                                            self.h1 = self.point2[2]
                                        else:
                                            break
                                    else:
                                        pass
                                else:
                                    modified_list.insert(0, (self.point2, self.r2, self.alpha2))
                            for i,value in enumerate(modified_list):
                                self.assignedList.append(value) 
                        else:
                            self.assignedList.append(self.point2)
                    else:
                        print("flagh1<h")#yes, h2 is greater
                        self.point2 = self.altitude_adjust_2(self.point2, self.r2, self.alpha2, self.point1,self.r1, self.alpha1) #here
                        if self.valid == True:
                            print("Flag is valid")
                            self.appending_to_list(self.point1, self.r1, self.alpha1)
                            self.appending_to_list(self.point2, self.r2, self.alpha2)
                            print("flag6",self.assignedList)
                        else:
                            print("Flag not valid")

                else:
                    print("flag OUT")
                    self.appending_to_list(self.point1, self.r1, self.alpha1)
                    self.appending_to_list(self.point2, self.r2, self.alpha2)
                    print ("self.assignedList",self.assignedList)
            print("Next loop")
        for i in range(len(self.assignedList)):
            Lx, Ly = self.calc_dimensions(self.assignedList[i][0][2], self.assignedList[i][2])
            print ("Lx, Ly",Lx, Ly)
            
            if (self.w + Lx) <= self.W:
                print("!")
                print("self.assignedList[i]",self.assignedList[i])
                self.boundedList.append(self.assignedList[i])  
                print("self.boundedList",self.boundedList)
                self.w += Lx   
                print("self.w",self.w)
            else:
                break
        print("self.boundedList",self.boundedList)
        return self.boundedList

