#!/usr/bin/env python3
import os
import sys
import math
import time
import yaml
import pygame
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from pygame.math import Vector2
from omni_carver_interfaces.srv import OmniCarverMode

# Resolve package share (or fall back to script location)
try:
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('joystick_pkg')
except Exception:
    script_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_share  = os.path.dirname(script_dir)

class JoystickUI(Node):
    def __init__(self):
        super().__init__('joystick_ui')

        # AMCL pose storage
        self.amcl_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_cb,
            10
        )

        # Teleop publisher & feedback
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.auto_lin_x = self.auto_lin_y = self.auto_ang_z = 0.0

        # Mode-change service
        self.mode_client = self.create_client(OmniCarverMode, 'omni_carver/mode')
        self.mode_client.wait_for_service()
        self.prev_mode = None

        # Nav2 Action client
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_client.wait_for_server()

        # Heartbeat subscription
        self.last_heartbeat = time.monotonic()
        self.create_subscription(Empty, 'omni_carver/heartbeat', self._hb_cb, 10)

        # Pygame setup
        pygame.init()
        self.screen = pygame.display.set_mode((1440, 960))
        pygame.display.set_caption('Omni Robot Teleop')
        self.clock  = pygame.time.Clock()
        self.font   = pygame.font.SysFont(None, 20)

        # Load map and metadata
        maps_dir = os.path.join(pkg_share, 'element')
        with open(os.path.join(maps_dir, 'map.yaml'), 'r') as f:
            cfg = yaml.safe_load(f)
        self.resolution = cfg['resolution']
        ox, oy, _ = cfg.get('origin', [0,0,0])
        self.origin_x, self.origin_y = ox, oy

        self.map_surf = pygame.image.load(os.path.join(maps_dir, cfg['image'])).convert()
        self.map_pos  = (200, 40)
        self.map_w, self.map_h = self.map_surf.get_size()

        # Teleop joystick state
        self.radius   = 80
        self.deadzone = 0.15
        self.center   = Vector2(200, 960-200)
        self.knob_pos = self.center.copy()
        self.drag     = False

        self.options   = [0.10, 0.15]
        self.max_ang   = self.options[-1]
        self.dd_rect   = pygame.Rect(375, self.center.y-self.radius-10, 100, 30)
        self.expanded  = False
        self.opt_rects = [pygame.Rect(375, self.dd_rect.y+30+i*30, 100, 30)
                          for i in range(len(self.options))]

        btn_y        = self.dd_rect.y + self.dd_rect.height + 70
        self.btn_left  = pygame.Rect(350, btn_y, 40, 40)
        self.btn_right = pygame.Rect(450, btn_y, 40, 40)
        self.btn_size  = 40
        self.btn_ang   = 0.0

        # Mode & autonomous controls
        x_mode, y_mode = 600, 650
        self.mode_dd_rect  = pygame.Rect(x_mode+50, y_mode+20, 200, 30)
        self.mode_options  = ['Teleoperation', 'Autonomous']
        self.mode_expanded = False
        self.mode_opt_rects = [pygame.Rect(x_mode+50, y_mode+20+(i+1)*30, 200, 30)
                                for i in range(len(self.mode_options))]
        self.mode = None

        self.btn_auto_goal = pygame.Rect(x_mode+50, y_mode+90,  200, 40)
        self.btn_auto_via  = pygame.Rect(x_mode+50, y_mode+140, 200, 40)
        self.btn_auto_nav  = pygame.Rect(x_mode+50, y_mode+190, 200, 40)

        # State for teleop & nav2
        self.lin_x = self.lin_y = 0.0

        # Single goal heading-define
        self.waiting_for_goal_click = False
        self.setting_goal_dir = False
        self.goal_start_px = None
        self.goal_pixel    = None
        self.goal_yaw      = None

        # Via-point with yaw
        self.waiting_for_via_click = False
        self.setting_via_dir = False
        self.via_start_px    = None
        self.via_pixels      = []
        self.via_yaws        = []
        self.via_poses       = []

        # Nav2 waypoint sequencing
        self._nav_handle_fut = None
        self._result_fut     = None
        self._nav_index      = 0
        self.nav_through_mode= False

    def _amcl_pose_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z))
        self.amcl_pose = (p.x, p.y, yaw)

    def _cmd_vel_cb(self, msg):
        self.auto_lin_x = msg.linear.x
        self.auto_lin_y = msg.linear.y
        self.auto_ang_z = msg.angular.z

    def _hb_cb(self, msg):
        self.last_heartbeat = time.monotonic()

    def run(self):
        while rclpy.ok():
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit(); rclpy.shutdown(); sys.exit()

                if ev.type == pygame.MOUSEBUTTONDOWN:
                    # Mode dropdown toggle
                    if self.mode_dd_rect.collidepoint(ev.pos):
                        self.mode_expanded = not self.mode_expanded
                        continue

                    # When expanded, show only the other mode (or both if none selected)
                    if self.mode_expanded:
                        opts = self.mode_options if self.mode is None else [m for m in self.mode_options if m != self.mode]
                        for opt in opts:
                            idx = self.mode_options.index(opt)
                            r = self.mode_opt_rects[idx]
                            if r.collidepoint(ev.pos):
                                self.mode = opt
                                self.mode_expanded = False
                                # reset teleop & pending flags
                                self.drag = False
                                self.knob_pos = self.center.copy()
                                self.btn_ang = 0.0
                                self.waiting_for_goal_click = False
                                self.setting_goal_dir    = False
                                self.waiting_for_via_click = False
                                self.setting_via_dir       = False
                        continue

                    # Single‐goal click handler
                    if (self.mode == 'Autonomous'
                        and self.btn_auto_goal.collidepoint(ev.pos)):
                        # ← clear everything on the map
                        self.goal_pixel      = None
                        self.via_pixels.clear()
                        self.via_poses.clear()
                        self.via_yaws.clear()

                        self.waiting_for_goal_click = True
                        self.get_logger().info('Click on map to send single Nav2 goal')
                        continue

                    # Map click to start heading drag
                    if self.waiting_for_goal_click:
                        x,y = ev.pos; mx,my = self.map_pos
                        if mx<=x<=mx+self.map_w and my<=y<=my+self.map_h:
                            px = x-mx; py = y-my
                            self.goal_start_px = (px,py)
                            self.setting_goal_dir = True
                        self.waiting_for_goal_click = False
                        continue

                    # Via-point: same two-step
                    if (self.mode=='Autonomous' and self.btn_auto_via.collidepoint(ev.pos)):
                        self.waiting_for_via_click = True
                        self.get_logger().info('Click-and-drag on map to add via + heading')
                        continue
                    if self.waiting_for_via_click:
                        x,y = ev.pos; mx,my = self.map_pos
                        if mx<=x<=mx+self.map_w and my<=y<=my+self.map_h:
                            px,py = x-mx, y-my
                            self.via_start_px = (px,py)
                            self.setting_via_dir = True
                        self.waiting_for_via_click = False
                        continue

                    # Start via navigation
                    if (self.mode=='Autonomous' and self.btn_auto_nav.collidepoint(ev.pos)
                        and self.via_poses):
                        self.nav_through_mode = True
                        self._nav_index = 0
                        goal = NavigateToPose.Goal(); goal.pose = self.via_poses[0]
                        self._nav_handle_fut = self.nav2_client.send_goal_async(goal)
                        self.get_logger().info('Navigating to waypoint 1')
                        continue

                    # Teleop knob/buttons
                    if self.mode=='Teleoperation':
                        mp = Vector2(ev.pos)
                        if mp.distance_to(self.center)<=self.radius:
                            self.drag = True
                        elif self.dd_rect.collidepoint(ev.pos):
                            self.expanded = not self.expanded
                        elif self.expanded:
                            for i,r in enumerate(self.opt_rects):
                                if r.collidepoint(ev.pos):
                                    self.max_ang = self.options[i]; self.expanded=False
                        elif self.btn_left.collidepoint(ev.pos):
                            self.btn_ang = -self.max_ang
                        elif self.btn_right.collidepoint(ev.pos):
                            self.btn_ang = +self.max_ang

                # Mouse up: finalize drag
                elif ev.type==pygame.MOUSEBUTTONUP:
                    # Finalize goal heading
                    if self.setting_goal_dir and self.goal_start_px is not None:
                        mx,my = self.map_pos
                        sx,sy = self.goal_start_px
                        cx,cy = mx+sx, my+sy
                        ex,ey = ev.pos
                        dx,dy = ex-cx, ey-cy
                        yaw = math.atan2(-dy, dx)
                        self.goal_yaw = yaw
                        # record as final pixel
                        self.goal_pixel = self.goal_start_px
                        # build and send goal
                        wx = sx*self.resolution + self.origin_x
                        wy = (self.map_h-sy)*self.resolution + self.origin_y
                        ps = PoseStamped()
                        ps.header.frame_id='map'; ps.header.stamp=self.get_clock().now().to_msg()
                        ps.pose.position.x=wx; ps.pose.position.y=wy
                        ps.pose.orientation.z = math.sin(yaw/2)
                        ps.pose.orientation.w = math.cos(yaw/2)
                        g = NavigateToPose.Goal(); g.pose=ps
                        self.nav2_client.send_goal_async(g)
                        self.get_logger().info(f"Sent goal x={wx:.2f},y={wy:.2f},yaw={yaw:.2f}")
                        self.setting_goal_dir=False
                        continue
                    # Finalize via heading
                    if self.setting_via_dir and self.via_start_px is not None:
                        mx,my = self.map_pos
                        sx,sy = self.via_start_px
                        cx,cy = mx+sx, my+sy
                        ex,ey = ev.pos
                        dx,dy = ex-cx, ey-cy
                        yaw = math.atan2(-dy, dx)
                        self.via_pixels.append((sx,sy))
                        self.via_yaws.append(yaw)
                        # send PoseStamped into via_poses
                        wx = sx*self.resolution + self.origin_x
                        wy = (self.map_h-sy)*self.resolution + self.origin_y
                        ps = PoseStamped(); ps.header.frame_id='map'; ps.header.stamp=self.get_clock().now().to_msg()
                        ps.pose.position.x=wx; ps.pose.position.y=wy
                        ps.pose.orientation.z=math.sin(yaw/2); ps.pose.orientation.w=math.cos(yaw/2)
                        self.via_poses.append(ps)
                        self.get_logger().info(f"Added via {len(self.via_poses)} x={wx:.2f},y={wy:.2f},yaw={yaw:.2f}")
                        self.setting_via_dir=False
                        continue
                    # Teleop release
                    if self.mode=='Teleoperation':
                        self.drag=False; self.knob_pos=self.center.copy(); self.btn_ang=0.0

                # Mouse motion: teleop knob drag
                elif ev.type==pygame.MOUSEMOTION and self.mode=='Teleoperation' and self.drag:
                    vec = Vector2(ev.pos)-self.center; vec.clamp_magnitude_ip(self.radius)
                    self.knob_pos = self.center+vec

            # Waypoint sequencing
            if self.nav_through_mode:
                if self._nav_handle_fut and self._nav_handle_fut.done():
                    gh=self._nav_handle_fut.result(); self._nav_handle_fut=None
                    self._result_fut=gh.get_result_async()
                elif self._result_fut and self._result_fut.done():
                    self._result_fut=None; self._nav_index+=1
                    if self._nav_index<len(self.via_poses):
                        goal=NavigateToPose.Goal(); goal.pose=self.via_poses[self._nav_index]
                        self._nav_handle_fut=self.nav2_client.send_goal_async(goal)
                        self.get_logger().info(f"Navigating to waypoint {self._nav_index+1}")
                    else:
                        self.get_logger().info('Finished all waypoints')
                        self.nav_through_mode=False

            # Mode-change service
            if self.mode in self.mode_options and self.mode!=self.prev_mode:
                req=OmniCarverMode.Request(); req.mode=self.mode
                self.mode_client.call_async(req); self.prev_mode=self.mode

            # Teleop velocity publish
            lin_x=lin_y=0.0
            if self.mode=='Teleoperation':
                disp=(self.knob_pos-self.center)/self.radius; disp.y=-disp.y
                xn=0 if abs(disp.x)<self.deadzone else disp.x
                yn=0 if abs(disp.y)<self.deadzone else disp.y
                lin_x=yn*(0.13 if yn>=0 else 0.15)
                lin_y=-xn*(0.13 if xn>=0 else 0.15)
            self.lin_x,self.lin_y=lin_x,lin_y
            t=Twist(); t.linear.x=lin_x; t.linear.y=lin_y
            t.angular.z=(self.btn_ang if self.mode=='Teleoperation' else 0.0)
            self.pub.publish(t)

            # Draw UI
            self.screen.fill((30,30,30))
            self.screen.blit(self.map_surf, self.map_pos)

            # AMCL pose
            if self.amcl_pose:
                mx,my=self.map_pos; w,h=self.map_w,self.map_h
                xw,yw,yaw=self.amcl_pose
                px=(xw-self.origin_x)/self.resolution
                py=h-(yw-self.origin_y)/self.resolution
                sx,sy=int(mx+px),int(my+py)
                pygame.draw.circle(self.screen,(0,255,0),(sx,sy),8,2)
                ex,ey=sx+20*math.cos(yaw), sy-20*math.sin(yaw)
                pygame.draw.line(self.screen,(0,255,0),(sx,sy),(ex,ey),2)

            # visualize drag
            if self.setting_goal_dir and self.goal_start_px:
                mx,my=self.map_pos; sx,sy=self.goal_start_px
                cx,cy=mx+sx,my+sy; ex,ey=pygame.mouse.get_pos()
                pygame.draw.circle(self.screen,(255,0,0),(cx,cy),8,1)
                pygame.draw.line(self.screen,(255,0,0),(cx,cy),(ex,ey),2)
            if self.setting_via_dir and self.via_start_px:
                mx,my=self.map_pos; sx,sy=self.via_start_px
                cx,cy=mx+sx,my+sy; ex,ey=pygame.mouse.get_pos()
                pygame.draw.circle(self.screen,(50,150,250),(cx,cy),6,1)
                pygame.draw.line(self.screen,(50,150,250),(cx,cy),(ex,ey),2)

            # final markers
            if self.goal_pixel and self.goal_yaw is not None:
                mx,my=self.map_pos; sx,sy=self.goal_pixel
                cx,cy=mx+sx,my+sy
                pygame.draw.circle(self.screen,(255,0,0),(cx,cy),8,2)
                ex,ey=cx+20*math.cos(self.goal_yaw), cy-20*math.sin(self.goal_yaw)
                pygame.draw.line(self.screen,(255,0,0),(cx,cy),(ex,ey),2)
            for idx,(px,py) in enumerate(self.via_pixels):
                mx,my=self.map_pos; cx,cy=mx+px,my+py
                yaw=self.via_yaws[idx]
                pygame.draw.circle(self.screen,(50,150,250),(cx,cy),6)
                ex,ey=cx+15*math.cos(yaw), cy-15*math.sin(yaw)
                pygame.draw.line(self.screen,(50,150,250),(cx,cy),(ex,ey),2)
                lbl=self.font.render(str(idx+1),True,(255,255,255))
                self.screen.blit(lbl,(cx-4,cy-8))

            # joystick grid + knob
            for ang in range(0,360,30):
                rad = math.radians(ang)
                end = (
                    self.center.x + self.radius*math.cos(rad),
                    self.center.y - self.radius*math.sin(rad)
                )
                pygame.draw.line(self.screen,(80,80,80),
                                 self.center,end,1)
            pygame.draw.circle(self.screen,(100,100,100),
                               self.center,self.radius,2)
            pygame.draw.circle(self.screen,(200,200,200),
                               self.knob_pos,16)
            for t,(lx,ly) in {
                '-Y':(self.center.x+self.radius+5, self.center.y-10),
                '+Y':(self.center.x-self.radius-25, self.center.y-10),
                '+X':(self.center.x-10, self.center.y-self.radius-20),
                '-X':(self.center.x-10, self.center.y+self.radius+5)
            }.items():
                self.screen.blit(self.font.render(t,True,(180,180,180)),(lx,ly))

            # ω-dropdown
            pygame.draw.rect(self.screen,(70,70,70),self.dd_rect)
            self.screen.blit(self.font.render(f"Max ω: {self.max_ang:.2f}",
                                             True,(220,220,220)),
                             (self.dd_rect.x+5,self.dd_rect.y+7))
            if self.expanded:
                for i, r in enumerate(self.opt_rects):
                    pygame.draw.rect(self.screen,(90,90,90),r)
                    self.screen.blit(self.font.render(f"{self.options[i]:.2f}",
                                                     True,(240,240,240)),
                                     (r.x+5,r.y+7))

            # angular buttons
            lc = (150,50,50) if self.btn_ang<0 else (120,120,120)
            rc = (50,150,50) if self.btn_ang>0 else (120,120,120)
            pygame.draw.rect(self.screen, lc,  self.btn_left)
            pygame.draw.rect(self.screen, rc,  self.btn_right)
            self.screen.blit(self.font.render("←",True,(255,255,255)),
                             (self.btn_left.x+12,self.btn_left.y+5))
            self.screen.blit(self.font.render("→",True,(255,255,255)),
                             (self.btn_right.x+12,self.btn_right.y+5))

            # --- 6) Velocity gauges ---
            if self.mode=='Teleoperation':
                gx_src, gy_src, ang_src = self.lin_x, self.lin_y, self.btn_ang
            else:
                gx_src, gy_src, ang_src = (
                    self.auto_lin_x,
                    self.auto_lin_y,
                    self.auto_ang_z
                )
            gw, gh = 150, 10
            gx = self.btn_left.x
            half = gw/2
            cx = gx + half
            gyx = self.btn_left.y + self.btn_size + 20

            # Lin X gauge
            pygame.draw.rect(self.screen,(100,100,100),(gx,gyx,gw,gh),2)
            if gx_src>=0:
                pygame.draw.rect(self.screen,(50,200,50),
                                 (cx,gyx,half*min(gx_src/0.13,1.0),gh))
            else:
                fw=half*min(abs(gx_src)/0.15,1.0)
                pygame.draw.rect(self.screen,(200,50,50),
                                 (cx-fw,gyx,fw,gh))
            self.screen.blit(self.font.render(f"Lin X: {gx_src:.2f}",
                                             True,(220,220,220)),
                             (gx+gw+10,gyx-2))

            # Lin Y gauge
            gyy=gyx+gh+10
            pygame.draw.rect(self.screen,(100,100,100),(gx,gyy,gw,gh),2)
            if gy_src>=0:
                pygame.draw.rect(self.screen,(50,200,50),
                                 (cx,gyy,half*min(gy_src/0.13,1.0),gh))
            else:
                fw=half*min(abs(gy_src)/0.15,1.0)
                pygame.draw.rect(self.screen,(200,50,50),
                                 (cx-fw,gyy,fw,gh))
            self.screen.blit(self.font.render(f"Lin Y: {gy_src:.2f}",
                                             True,(220,220,220)),
                             (gx+gw+10,gyy-2))

            # Angular gauge (split)
            gya=gyy+gh+10
            pygame.draw.rect(self.screen,(100,100,100),(gx,gya,gw,gh),2)
            frac = (min(abs(ang_src)/self.max_ang,1.0) if self.max_ang else 0.0)
            cx = gx + gw/2
            if ang_src>=0:
                pygame.draw.rect(self.screen,(50,200,50),
                                 (cx,gya,(gw/2)*frac,gh))
            else:
                pygame.draw.rect(self.screen,(200,50,50),
                                 (cx-(gw/2)*frac,gya,(gw/2)*frac,gh))
            self.screen.blit(self.font.render(f"Ang: {ang_src:.2f}",
                                             True,(220,220,220)),
                             (gx+gw+10,gya-2))

            # dim teleop when inactive
            if self.mode!='Teleoperation':
                pad=10
                jr=pygame.Rect(self.center.x-self.radius-pad,
                               self.center.y-self.radius-pad,
                               2*self.radius+2*pad,2*self.radius+2*pad)
                s=pygame.Surface((jr.width,jr.height),pygame.SRCALPHA)
                s.fill((50,50,50,180))
                self.screen.blit(s,jr.topleft)

                left = min(self.dd_rect.x,self.btn_left.x)-pad
                top  = self.dd_rect.y-pad
                right= max(self.dd_rect.right,self.btn_right.right)+pad
                bot  = self.btn_left.bottom+pad
                cr=pygame.Rect(left,top,right-left,bot-top)
                s2=pygame.Surface((cr.width,cr.height),pygame.SRCALPHA)
                s2.fill((50,50,50,180))
                self.screen.blit(s2,cr.topleft)

            # --- 7) Mode selection & Autonomous buttons ---
            # Draw mode dropdown label & options
            # Draw mode dropdown label & options
            self.screen.blit(self.font.render("Mode selection:", True, (255,255,255)),
                             (self.mode_dd_rect.x, self.mode_dd_rect.y-20))
            pygame.draw.rect(self.screen, (70,70,70), self.mode_dd_rect)
            label = self.mode if self.mode is not None else "Choose.."
            self.screen.blit(self.font.render(label, True, (220,220,220)),
                             (self.mode_dd_rect.x+5, self.mode_dd_rect.y+7))

            if self.mode_expanded:
                opts = self.mode_options if self.mode is None else [m for m in self.mode_options if m != self.mode]
                for opt in opts:
                    idx = self.mode_options.index(opt)
                    r = self.mode_opt_rects[idx]
                    pygame.draw.rect(self.screen, (90,90,90), r)
                    self.screen.blit(self.font.render(opt, True, (240,240,240)),
                                     (r.x+5, r.y+5))


            if self.mode=='Autonomous':
                for btn,txt in [
                    (self.btn_auto_goal,"Goal pose"),
                    (self.btn_auto_via,"Set via point"),
                    (self.btn_auto_nav,"Navigate")
                ]:
                    pygame.draw.rect(self.screen,(80,80,80),btn)
                    w = self.font.size(txt)[0]
                    self.screen.blit(self.font.render(txt,True,(255,255,255)),
                                     (btn.x+(btn.width-w)//2,btn.y+10))

            # --- 8) Heartbeat indicator ---
            hb_txt = self.font.render("Heartbeat:", True, (255,255,255))
            hb_x = self.mode_dd_rect.x + self.mode_dd_rect.width + 20
            hb_y = self.mode_dd_rect.y - 20
            self.screen.blit(hb_txt, (hb_x, hb_y))
            alive = (time.monotonic() - self.last_heartbeat) < 7.0
            color = (50,200,50) if alive else (200,50,50)
            center = (
                hb_x + hb_txt.get_width() + 12,
                hb_y + hb_txt.get_height()//2
            )
            pygame.draw.circle(self.screen, color, center, 8)

            pygame.display.flip()
            self.clock.tick(60)
            rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    JoystickUI().run()


if __name__ == '__main__':
    main()
