import yaml

class PutziniConfig:

    def __init__(self):
        self.range_x = (0, 0)
        self.range_y = (0, 0)
        self.anchor_names = ('a', 'b', 'c')
        self.tag_name = 'd'
        self.anchor_x = (10, 20, 30)
        self.anchor_y = (40, 50, 60)
        self.waypoint_x = (10, 20, 30)
        self.waypoint_y = (40, 50, 60)
        self.nav_update_rate = 20
        self.bno055_calib = {'a': 1, 'b': 2, 'c': 3}
        self.room_rotation = 0
        self.arka_radius = 100
        self.keepout_img = 'keepout.tiff'
        self.minimum_calib_level = 0
        try:
            self.from_yaml()
        except FileNotFoundError:
            print('putzini.yaml not found, initializing one...')
            self.to_yaml()

    def to_yaml(self):
        opts = {k: v for k, v in self.__dict__.items() if not k.startswith('_')}
        print(opts)
        yaml.dump(opts, open('putzini.yaml', 'w'))

    def from_yaml(self):
        opts = yaml.load(open('putzini.yaml', 'r'))
        print(opts)
        for k, v in opts.items():
            if hasattr(self, k):
                setattr(self, k, v)
            else:
                print(f'Option {k} in yaml file is not recognized.')

