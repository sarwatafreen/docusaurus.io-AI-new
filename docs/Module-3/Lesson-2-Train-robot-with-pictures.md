---
sidebar_position: 2
---

# Lesson 2: Train Robot with 10,000 Pictures in 5 Minutes

In this lesson, we'll leverage Isaac Sim's synthetic data generation capabilities to create thousands of training images for our robot's perception systems. This approach significantly reduces the need for real-world data collection.

## Prerequisites
- Completed Module 3, Lesson 1
- NVIDIA Isaac Sim with Omniverse Kit
- Basic understanding of computer vision
- Python development environment

## What You'll Build
By the end of this lesson, you'll have:
1. Configured Isaac Sim for synthetic data generation
2. Set up domain randomization for robust training
3. Generated 10,000+ labeled training images in 5 minutes
4. Trained a perception model using synthetic data

## Understanding Synthetic Data Generation

Synthetic data generation creates labeled training data in simulation rather than collecting it from the real world. Benefits include:

- Rapid dataset creation without physical limitations
- Perfect ground truth labels
- Controlled environmental conditions
- Domain randomization for robust models

## Setting Up the Synthetic Data Pipeline

In Isaac Sim, we'll use the Replicator framework to generate synthetic data:

```python
import omni.replicator.core as rep

# Initialize the replicator
with rep.new_layer():
    # Define the camera to capture images
    camera = rep.create.camera(
        position=(-1, -1, 2),
        look_at=(0, 0, 0)
    )
    
    # Define lighting
    lights = rep.create.light(
        light_type="dome",
        texture_brightness=100,
        enabled=True,
    )
    
    # Define objects to randomize
    objects = rep.get.prims_from_path("/World/Objects")
    
    # Randomize object positions
    with objects.randomize():
        rep.randomizer.attribute("xformOp:translate", 
                                value=rep.distribution.uniform((-1, -1, 0), (1, 1, 0)))
    
    # Randomize material properties
    materials = rep.get.materials()
    with materials.randomize():
        rep.randomizer.attribute("mdl:argument:diffuse_tint", 
                                value=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
```

## Configuring Domain Randomization

Domain randomization helps models generalize to the real world by varying environmental properties:

```python
# Randomize lighting conditions
with rep.trigger.on_time(interval=5):
    lights.env_range = (0.5, 1.5)
    lights.temperature_range = (4000, 8000)

# Randomize textures and materials
with rep.randomizer.on_object_signal(obj=objects, signal_name="randomize_materials", frequency=1):
    rep.randomizer.apply_material(
        materials=rep.distribution.choice(
            [rep.mats.chrome_albedo(), rep.mats.plastic_albedo(), rep.mats.metal_albedo()]
        ),
        objects=objects
    )
```

## Generating Labeled Datasets

Isaac Sim provides various ground truth annotations:

```python
# Define the annotations we want
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="output", rgb=True, semantic_segmentation=True, 
                  instance_segmentation=True, bounding_box_2d_tight=True)

# Create capture triggers
with rep_trigger.on_frame(num_frames=10000):
    writer.write()
```

## Training a Perception Model

With our synthetic dataset, we can train a perception model:

```bash
# Using a standard object detection framework like Detectron2
python train_model.py --dataset_path ./output --model_config config.yaml
```

## Optimizing Data Generation

To achieve 10,000+ images in 5 minutes:

1. Utilize GPU-accelerated rendering
2. Parallelize data generation across multiple environments
3. Optimize the rendering pipeline
4. Use efficient storage formats

## Quiz

1. What is domain randomization and why is it important?
2. Name three types of ground truth annotations Isaac Sim can provide
3. What are the main benefits of synthetic data over real-world data?

## Summary

In this lesson, you learned to generate thousands of labeled training images using Isaac Sim's Replicator framework. You configured domain randomization and synthetic data pipelines that can produce datasets much faster than real-world collection. This synthetic data will power your robot's perception systems! In the next lesson, we'll implement VSLAM to enable autonomous mapping.