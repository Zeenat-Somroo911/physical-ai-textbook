---
sidebar_position: 3
---

# Chapter 2: Multimodal Learning

## Introduction

Multimodal learning is the core of VLA systems, enabling models to understand and reason about information from multiple modalities simultaneously.

## Multimodal Representations

### Early Fusion

Combine modalities at input level:

```python
import torch
import torch.nn as nn

class EarlyFusion(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.fusion = nn.Linear(vision_dim + lang_dim, hidden_dim)
    
    def forward(self, image, text):
        v_features = self.vision_encoder(image)
        l_features = self.language_encoder(text)
        combined = torch.cat([v_features, l_features], dim=-1)
        fused = self.fusion(combined)
        return fused
```

### Late Fusion

Fuse after encoding:

```python
class LateFusion(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.cross_attention = nn.MultiheadAttention(hidden_dim, num_heads=8)
    
    def forward(self, image, text):
        v_features = self.vision_encoder(image)
        l_features = self.language_encoder(text)
        
        # Cross-attention
        fused, _ = self.cross_attention(
            v_features, l_features, l_features
        )
        return fused
```

## Cross-Modal Attention

### Attention Mechanism

```python
class CrossModalAttention(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.query = nn.Linear(dim, dim)
        self.key = nn.Linear(dim, dim)
        self.value = nn.Linear(dim, dim)
    
    def forward(self, vision, language):
        q = self.query(vision)
        k = self.key(language)
        v = self.value(language)
        
        scores = torch.matmul(q, k.transpose(-2, -1))
        attention = torch.softmax(scores, dim=-1)
        output = torch.matmul(attention, v)
        return output
```

## CLIP-style Learning

### Contrastive Learning

```python
import torch.nn.functional as F

def clip_loss(vision_features, language_features, temperature=0.07):
    # Normalize
    vision_features = F.normalize(vision_features, dim=-1)
    language_features = F.normalize(language_features, dim=-1)
    
    # Compute similarity
    logits = torch.matmul(vision_features, language_features.T) / temperature
    
    # Contrastive loss
    labels = torch.arange(len(vision_features))
    loss = F.cross_entropy(logits, labels)
    return loss
```

## Transformer-based Fusion

### Multimodal Transformer

```python
from transformers import BertModel, ViTModel

class MultimodalTransformer(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = ViTModel.from_pretrained('google/vit-base')
        self.language_encoder = BertModel.from_pretrained('bert-base')
        
        # Fusion transformer
        self.fusion_layers = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=768, nhead=12),
            num_layers=6
        )
    
    def forward(self, image, text):
        v_features = self.vision_encoder(image).last_hidden_state
        l_features = self.language_encoder(text).last_hidden_state
        
        # Concatenate and fuse
        combined = torch.cat([v_features, l_features], dim=1)
        fused = self.fusion_layers(combined)
        return fused
```

## Perceiver Architecture

### Cross-Attention Perceiver

```python
class PerceiverVL(nn.Module):
    def __init__(self):
        super().__init__()
        self.latent = nn.Parameter(torch.randn(256, 768))
        self.cross_attn = nn.MultiheadAttention(768, num_heads=12)
        self.self_attn = nn.TransformerEncoderLayer(768, nhead=12)
    
    def forward(self, vision, language):
        # Cross-attend to inputs
        latent, _ = self.cross_attn(
            self.latent,
            torch.cat([vision, language], dim=1),
            torch.cat([vision, language], dim=1)
        )
        
        # Self-attention
        output = self.self_attn(latent)
        return output
```

## Training Strategies

### Pre-training

1. **Vision-Language Pre-training**: Learn general representations
2. **Task-specific Fine-tuning**: Adapt to robotics

### Multi-task Learning

```python
class MultiTaskVLA(nn.Module):
    def __init__(self):
        super().__init__()
        self.shared_encoder = MultimodalEncoder()
        self.action_head = ActionHead()
        self.vqa_head = VQAHead()
    
    def forward(self, image, text, task='action'):
        features = self.shared_encoder(image, text)
        
        if task == 'action':
            return self.action_head(features)
        elif task == 'vqa':
            return self.vqa_head(features)
```

## Best Practices

- Use pre-trained encoders when possible
- Align feature spaces across modalities
- Use appropriate fusion strategies
- Regularize to prevent overfitting

## Exercises

1. Implement early fusion model
2. Add cross-modal attention
3. Train on multimodal dataset

## Next Steps

Chapter 3 covers language grounding in visual scenes.

