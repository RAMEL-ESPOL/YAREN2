import os
from data_processor import DataProcessor
from visualization import Visualizer
from config import Config

def explore_dataset():
    config = Config()
    data_processor = DataProcessor()
    visualizer = Visualizer()
    
    if not os.path.exists(config.DATA_PATH):
        print(f"ERROR: The file {config.DATA_PATH} does not exist.")
        return
    
    print("1. Loading dataset...")
    df = data_processor.load_data()
    
    print("\n2. Basic Dataset Information...")
    print(f"   - Number of images: {len(df)}")
    print(f"   - Size of dataset: {df.shape}")
    print(f"   - Unique classes: {df.emotion.nunique()}")
    print(f"   - Number of emotions: {len(config.EMOTION_LABELS)}")
    print(f"   - Columns: {list(df.columns)}")
    print(f"   - Types of data:\n{df.dtypes.to_dict()}")
    
    print("\n3. Statistics of Dataset Emotions...")
    emotion_stats = df.emotion.describe()
    print(emotion_stats)
    
    print("\n4. Distribution of Emotions...")
    analyze_emotion_balance(df)

    print("\n5. Analyzing Pixel Statistics...")
    analyze_pixel_statistics(df)
    
    print("\n6. Generating Visualizations...")
    visualizer.plot_emotion_distribution(df)
    visualizer.plot_sample_images(df, samples_per_emotion=5)
    
    print("\nAnalysis complete!")

def analyze_emotion_balance(df = None):
    config = Config()

    if df is None:
        data_processor = DataProcessor()
        df = data_processor.load_data()
    
    emotion_counts = df.emotion.value_counts().sort_index()
    total_images = len(df)
    
    print("Classes Balance Analysis:")
    print("-" * 50)
    
    for emotion_id, count in emotion_counts.items():
        emotion_name = config.EMOTION_LABELS[emotion_id]
        percentage = (count / total_images) * 100
        bar_length = int(percentage / 2)  
        bar = "█" * bar_length + "░" * (50 - bar_length)
        
        print(f"{emotion_name.capitalize():<10} │{bar}│ {count:>5} ({percentage:>5.1f}%)")
    
    max_count = emotion_counts.max()
    min_count = emotion_counts.min()
    imbalance_ratio = max_count / min_count
    
    print(f"\nMetrics of Class Balance:")
    print(f"- Class more frequent: {emotion_counts.idxmax()} with {max_count} images")
    print(f"- Class less frequent: {emotion_counts.idxmin()} with {min_count} images")
    print(f"- Ratio of imbalance: {imbalance_ratio:.2f}")

def analyze_pixel_statistics(df = None):
    config = Config()

    if df is None:
        data_processor = DataProcessor()
        df = data_processor.load_data()
        
    sample_size = min(1000, len(df))
    sample_df = df.sample(n=sample_size, random_state=42)
    
    pixel_stats = []
    
    for idx, row in sample_df.iterrows():
        pixels = [int(p) for p in row.pixels.split(' ')]
        pixel_stats.append({
            'mean': sum(pixels) / len(pixels),
            'min': min(pixels),
            'max': max(pixels),
            'std': (sum([(p - sum(pixels) / len(pixels))**2 for p in pixels]) / len(pixels))**0.5
        })
    
    overall_stats = {
        'mean_brightness': sum([s['mean'] for s in pixel_stats]) / len(pixel_stats),
        'avg_std': sum([s['std'] for s in pixel_stats]) / len(pixel_stats),
        'min_pixel': min([s['min'] for s in pixel_stats]),
        'max_pixel': max([s['max'] for s in pixel_stats])
    }
    
    print(f"Statistics of pixels (sample of {sample_size} images):")
    print(f"- Brightness mean: {overall_stats['mean_brightness']:.2f}")
    print(f"- Standard deviation of brightness: {overall_stats['avg_std']:.2f}")
    print(f"- Minimum pixel value: {overall_stats['min_pixel']}")
    print(f"- Maximum pixel value: {overall_stats['max_pixel']}")
    
    print("\nStatistics by emotion:")
    print("Emotion        │ Brightness Mean")
    print("-" * 60)
    
    for emotion_id in sorted(df.emotion.unique()):
        emotion_name = config.EMOTION_LABELS[emotion_id]
        emotion_data = sample_df[sample_df.emotion == emotion_id]
        
        if len(emotion_data) > 0:
            emotion_pixels = []
            for _, row in emotion_data.iterrows():
                pixels = [int(p) for p in row.pixels.split(' ')]
                emotion_pixels.extend(pixels)
            
            if emotion_pixels:
                avg_brightness = sum(emotion_pixels) / len(emotion_pixels)
                print(f"{emotion_name.capitalize():<12} │ {avg_brightness:.2f}")

def main():
    print("Select an analysis option:")
    print("1. Exploratory dataset analysis")
    print("2. Balance class analysis")
    print("3. Statistics analysis of pixels")
    
    choice = input("\Select your option (1-3): ").strip()
    
    if choice == "1":
        explore_dataset()
    elif choice == "2":
        analyze_emotion_balance()
    elif choice == "3":
        analyze_pixel_statistics()
    else:
        print("Invalid option. Please try again.")

if __name__ == "__main__":
    main()