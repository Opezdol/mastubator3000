from dataclasses import dataclass

from build123d import *  # type:ignore

## define GLOBALS of the model
BLADE_N4_PTS = [(5, 0), (75, -35), (145, 10)]
D_BLADE_PTS = [
    (0, 0),
    (75, -15),
]


@dataclass
class Base_Blade:
    wire: Spline
    width = 25
    blade_thick = 18
    cam_r = 8
    cam_l = 44.7
    cam_offset = 1.5  # both sides
    r = 2
    hand_w = 45

    # calc props ZONE
    @property
    def blade_lip_width(self):
        return self.cam_r + self.cam_offset * 2

    ## We build sketches for bodies

    def sk_v(self, scalar: float = 1, r: float = 2, offsetY:float = 3) -> Sketch:
        cal_blade_thick = self.blade_thick * scalar

        mv = -(self.width / 2) + self.blade_lip_width / 2

        sk= RectangleRounded(
            self.blade_lip_width,
            cal_blade_thick - offsetY,
            r,
            align=(Align.CENTER, Align.MIN, Align.CENTER),  # type:ignore
        )
        res = Pos(X=-mv) * sk
        return Pos(Y=-offsetY) * res

    def sk_h(
        self, scalar: float = 1 , r: float = 1, w_scale: float = 1
    ) -> Sketch:

        cal_blade_thick = self.blade_thick * scalar
        res = RectangleRounded(self.width * w_scale, cal_blade_thick, r,
            align=(Align.CENTER, Align.MIN, Align.CENTER))  # type:ignore
        return res


    def sk_init(self, r:float = 4, scalar: float = 2)->Sketch:
        hor_sketch = self.sk_h(scalar=scalar,r=r)
        return hor_sketch


    def sk_body(self, scale_v, scale_h, inner_fillet = 2, outer_fillet = 2):
        hor_sketch = self.sk_h(scale_h)
        ver_sketch = self.sk_v(scale_v)
        ## Verts manipulation
        vert_snapshot = hor_sketch.vertices() + ver_sketch.vertices()
        base_no_fillet = hor_sketch + ver_sketch
        new_verts = base_no_fillet.clean().vertices() - vert_snapshot
        v_inner = new_verts.sort_by(Axis.Y)[1]
        ## sketchz zone
        sk_inner_fillet = fillet(v_inner, inner_fillet)
        sk_outer_fillet = fillet(base_no_fillet.vertices() - v_inner, outer_fillet)

        show_object(sk_outer_fillet & sk_inner_fillet, name="sk_outer_fillet")
        show_object(sk_inner_fillet & sk_outer_fillet, name="sk_inner_fillet")
        return base_no_fillet

    def tip(self)->Part:
        tip_body = Cylinder(radius=0.15*self.blade_thick, height=0.69 * self.width, rotation=(90,0,0),align=(Align.CENTER,Align.CENTER, Align.CENTER))
        tip_filleted = chamfer(tip_body.edges(), 0.5)
        return (self.wire ^ 1) *tip_filleted        #type:ignore

    def blade_h(self) -> Part:
        """
        Loft horizontal blade && tip 
        """
        show_object(self.wire, name='wire')
        rrt = Rot(Z=-90)
        body = [
            (self.wire ^ 0)  *rrt * self.sk_init(),
            (self.wire ^ 0.25) * rrt * self.sk_h(0.5),
            (self.wire ^ 0.85) *rrt *  self.sk_h(0.3),
            (self.wire ^ 1) *rrt *  self.sk_h(0.15, w_scale=0.69),
        ]
        res = loft(body) + self.tip()
        return res

    def blade_v(self) -> Part:
        rrt = Rot(Z=-90)
        body = [
            (self.wire ^ 0)  *rrt * self.sk_init(),
            (self.wire ^ 0.25) * rrt * self.sk_v(),
            (self.wire ^ 0.6) * rrt * self.sk_v(scalar=0.5, offsetY=0, r=1),
            (self.wire ^ 0.85) * rrt * self.sk_v(scalar=0.1, offsetY=0, r=0.2),
        ]
        res = loft(body)
        return res


    def cam(self, offline:float = 5):
        """
        I have to use complicated way to place cam body on the wire
        """
        cam_wire_r = 3.62/2
        cam_r = 5.65/2
        cam_lip = cam_r - 0.75
        pos_v = self.wire @ 0.8
        rot_v = self.wire % 0.7
        cam_wire_inset =  cam_wire_r/3



        wha = self.wire ^ 0.6
        print(f"_____\nPos \t{pos_v}\nRot{rot_v}______\n")

        x_offset = self.width / 2 - self.blade_lip_width / 2
        sk= Pos(Y=-x_offset, Z=self.cam_l* 0.69) * Circle(cam_r)
        cam_wire_body = wha * Pos(X=-offline) * sk
        cam_body = extrude(cam_wire_body, amount=-self.cam_l, taper=-1)
        cam_face = cam_body.faces()[-1]
        cam_body += Plane(cam_face) * extrude(Circle(cam_lip),amount=10, taper=2)
        ## Incision for cam wire
        incision  = Plane(cam_face) * extrude(Rectangle(10, cam_wire_r*2, align=(Align.MAX,Align.CENTER)), amount=-self.cam_l)
        cam_body += incision
        #show_object(cam_face, name='cam_face')
        cam_wire_body = [
            (self.wire ^ 0)  * Pos( X= cam_wire_inset) * Circle(cam_wire_r),
            (self.wire ^ 0.25)  * Pos(Y=-x_offset, X =-4 +cam_wire_inset) * Circle(cam_wire_r),
            (self.wire ^ 0.4)  * Pos(Y=-x_offset , X = 0  + cam_wire_inset) * Circle(cam_wire_r),
            wha * Pos( X =cam_wire_inset,  Y=-x_offset,Z =0)  * Circle(4/2),  ## -self.cam_l/4)
        ]
        cam_wire_body = loft(cam_wire_body)

        addendum = extrude((self.wire ^ 0)  * Pos( X= cam_wire_inset) * Circle(cam_wire_r), amount=-30, taper=-0.5)
        #show_object(addendum, name="addendum")
        return cam_body + cam_wire_body + addendum

    def sk_hand(self,finged:bool = False ) -> Sketch:

        res = RectangleRounded(self.hand_w, self.width, 4,
            align=(Align.MIN, Align.CENTER, Align.CENTER))  # type:ignore
        if(finged):
            fing_incision = 5
            res = Pos(X=-fing_incision) * RectangleRounded(self.hand_w - fing_incision, self.width, 4,
                align=(Align.MIN, Align.CENTER, Align.CENTER))  # type:ignore
        return res


    def hand(self):
        pnts = [
            (5,0),
            (-5,30),
            (-10,50),
            (5,100),
        ]
        hand_wire = Spline(pnts)
        show_object(hand_wire, name="hand_wire")
        body = [
            (self.wire ^ 0)  *Rot(Z=-90) * self.sk_init(),
            (hand_wire ^ 0.2) *   self.sk_hand(finged = True),
            (hand_wire ^ 0.5) *  self.sk_hand(finged =False),
            #(hand_wire ^ 0.65)  *  self.sk_hand(finged = False),
            (hand_wire ^ 0.85) *  self.sk_hand(finged = True),
            (hand_wire ^ 1) *   self.sk_hand(finged = False),
        ]
        res = loft(body)
        return res

    def build(self):
        """
        Build the blade by defining different 2D "Sketch" positions on given curve aka his majesty-"self.wire"
        """
        # dev zone
        blade = self.blade_h() + self.blade_v() + self.hand()
        cam = self.cam(offline=-2.3)
        blade -= cam
        show_object(cam, name="cam")
        return blade


blade_spline = Spline(BLADE_N4_PTS)
test = Base_Blade(wire=blade_spline)
show_object(test.hand(), name="hand")
show_object(test.build(), name="build")

