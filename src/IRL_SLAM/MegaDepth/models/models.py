from MegaDepth.options.train_options import TrainOptions
def create_model():
    model = None
    from .HG_model import HGModel
    model = HGModel()
    print("model [%s] was created" % (model.name()))
    return model
