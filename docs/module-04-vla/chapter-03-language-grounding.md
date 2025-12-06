---
sidebar_position: 4
---

# Chapter 3: Language Grounding

## Introduction

Language grounding connects natural language instructions to visual scenes, enabling robots to understand what to do and where to do it.

## What is Language Grounding?

Language grounding involves:
- **Object grounding**: Identifying objects mentioned in language
- **Spatial grounding**: Understanding spatial relationships
- **Action grounding**: Mapping language to actions
- **Temporal grounding**: Understanding sequences and timing

## Object Grounding

### Visual Object Detection

```python
import torch
import torch.nn as nn

class ObjectGrounding(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.object_detector = ObjectDetector()
    
    def forward(self, image, text):
        # Extract object mentions from text
        objects = extract_objects(text)  # ["cup", "table"]
        
        # Detect objects in image
        detections = self.object_detector(image)
        
        # Match language to visual objects
        matches = match_objects(objects, detections)
        return matches
```

### Referring Expression Comprehension

```python
class ReferringExpression(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = MultimodalEncoder()
        self.scorer = nn.Linear(hidden_dim, 1)
    
    def forward(self, image, expression):
        # "the red cup on the table"
        features = self.encoder(image, expression)
        
        # Score all regions
        scores = self.scorer(features)
        best_region = torch.argmax(scores)
        return best_region
```

## Spatial Grounding

### Spatial Relationship Understanding

```python
class SpatialGrounding(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = MultimodalEncoder()
        self.spatial_parser = SpatialParser()
    
    def forward(self, image, instruction):
        # "pick up the cup on the left side of the table"
        features = self.encoder(image, instruction)
        
        # Parse spatial relationships
        relationships = self.spatial_parser(features)
        # Returns: {cup: {position: left, relative_to: table}}
        return relationships
```

### Spatial Attention

```python
class SpatialAttention(nn.Module):
    def __init__(self):
        super().__init__()
        self.attention = nn.MultiheadAttention(dim, num_heads=8)
    
    def forward(self, image_features, spatial_query):
        # spatial_query: "left", "right", "above", etc.
        attended, weights = self.attention(
            image_features, spatial_query, spatial_query
        )
        return attended, weights
```

## Action Grounding

### Action-Object Mapping

```python
class ActionGrounding(nn.Module):
    def __init__(self):
        super().__init__()
        self.action_classifier = nn.Linear(hidden_dim, num_actions)
        self.object_localizer = ObjectLocalizer()
    
    def forward(self, image, instruction):
        # "pick up the cup"
        # Extract action: "pick up"
        # Extract object: "cup"
        
        action = classify_action(instruction)
        object_location = self.object_localizer(image, "cup")
        
        return {
            'action': action,
            'object': object_location,
            'target_pose': compute_grasp_pose(object_location)
        }
```

## Temporal Grounding

### Sequence Understanding

```python
class TemporalGrounding(nn.Module):
    def __init__(self):
        super().__init__()
        self.lstm = nn.LSTM(hidden_dim, hidden_dim, num_layers=2)
        self.temporal_parser = TemporalParser()
    
    def forward(self, video_frames, instruction):
        # "first pick up the cup, then place it on the table"
        features = [self.encoder(frame, instruction) for frame in video_frames]
        
        # Understand temporal sequence
        sequence, _ = self.lstm(torch.stack(features))
        parsed = self.temporal_parser(sequence)
        return parsed
```

## Grounding Datasets

### Popular Datasets

- **RefCOCO/RefCOCO+**: Referring expressions
- **Flickr30k Entities**: Visual grounding
- **Visual Genome**: Scene understanding
- **ALFRED**: Language-conditioned tasks

## Training Grounding Models

### Weakly Supervised Learning

```python
def grounding_loss(predictions, weak_labels):
    # Weak labels: object present/not present
    # No exact bounding boxes
    loss = F.binary_cross_entropy(predictions, weak_labels)
    return loss
```

### Contrastive Learning

```python
def contrastive_grounding_loss(positive_pairs, negative_pairs):
    # Positive: (image, correct_text)
    # Negative: (image, incorrect_text)
    
    pos_sim = cosine_similarity(positive_pairs)
    neg_sim = cosine_similarity(negative_pairs)
    
    loss = -torch.log(torch.exp(pos_sim) / (torch.exp(pos_sim) + torch.exp(neg_sim)))
    return loss
```

## Evaluation Metrics

- **Accuracy**: Correct object identification
- **IoU**: Intersection over Union for bounding boxes
- **Success Rate**: Task completion rate
- **Grounding Precision**: Correct spatial grounding

## Best Practices

- Use pre-trained vision-language models
- Leverage spatial reasoning modules
- Train with diverse instructions
- Evaluate on real-world scenarios

## Exercises

1. Implement object grounding
2. Add spatial relationship parsing
3. Evaluate on grounding dataset

## Next Steps

Chapter 4 covers action prediction from language and vision.

