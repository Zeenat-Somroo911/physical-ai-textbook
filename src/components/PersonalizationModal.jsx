import React, { useState } from 'react';
import styles from './PersonalizationModal.module.css';

/**
 * PersonalizationModal Component
 * 
 * Two-step modal for selecting personalization preferences:
 * Step 1: Language (Urdu, English, Roman Urdu)
 * Step 2: Difficulty (Easy, Medium, Hard)
 */
export default function PersonalizationModal({ isOpen, onClose, onSubmit }) {
    const [step, setStep] = useState(1);
    const [language, setLanguage] = useState('english');
    const [difficulty, setDifficulty] = useState('medium');

    if (!isOpen) return null;

    const handleNext = () => {
        if (step === 1) {
            setStep(2);
        } else {
            // Submit preferences
            onSubmit({ language, difficulty });
            handleClose();
        }
    };

    const handleBack = () => {
        if (step === 2) {
            setStep(1);
        }
    };

    const handleClose = () => {
        // Reset to initial state
        setStep(1);
        setLanguage('english');
        setDifficulty('medium');
        onClose();
    };

    return (
        <div className={styles.backdrop} onClick={handleClose}>
            <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
                {/* Step Indicator */}
                <div className={styles.stepIndicator}>
                    Step {step} of 2
                </div>

                {/* Modal Header */}
                <h2 className={styles.title}>
                    {step === 1 ? 'Select Language' : 'Select Difficulty'}
                </h2>

                {/* Step 1: Language Selection */}
                {step === 1 && (
                    <div className={styles.optionsContainer}>
                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="language"
                                value="english"
                                checked={language === 'english'}
                                onChange={(e) => setLanguage(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>🇬🇧</span>
                                <span className={styles.optionText}>
                                    <strong>English</strong>
                                    <small>Standard English content</small>
                                </span>
                            </span>
                        </label>

                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="language"
                                value="urdu"
                                checked={language === 'urdu'}
                                onChange={(e) => setLanguage(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>🇵🇰</span>
                                <span className={styles.optionText}>
                                    <strong>Urdu</strong>
                                    <small>اردو زبان میں مواد</small>
                                </span>
                            </span>
                        </label>

                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="language"
                                value="roman_urdu"
                                checked={language === 'roman_urdu'}
                                onChange={(e) => setLanguage(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>📝</span>
                                <span className={styles.optionText}>
                                    <strong>Roman Urdu</strong>
                                    <small>Urdu in Roman script</small>
                                </span>
                            </span>
                        </label>
                    </div>
                )}

                {/* Step 2: Difficulty Selection */}
                {step === 2 && (
                    <div className={styles.optionsContainer}>
                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="difficulty"
                                value="easy"
                                checked={difficulty === 'easy'}
                                onChange={(e) => setDifficulty(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>🌱</span>
                                <span className={styles.optionText}>
                                    <strong>Easy</strong>
                                    <small>Beginner-friendly explanations</small>
                                </span>
                            </span>
                        </label>

                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="difficulty"
                                value="medium"
                                checked={difficulty === 'medium'}
                                onChange={(e) => setDifficulty(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>⚡</span>
                                <span className={styles.optionText}>
                                    <strong>Medium</strong>
                                    <small>Balanced technical depth</small>
                                </span>
                            </span>
                        </label>

                        <label className={styles.option}>
                            <input
                                type="radio"
                                name="difficulty"
                                value="hard"
                                checked={difficulty === 'hard'}
                                onChange={(e) => setDifficulty(e.target.value)}
                            />
                            <span className={styles.optionLabel}>
                                <span className={styles.optionIcon}>🚀</span>
                                <span className={styles.optionText}>
                                    <strong>Hard</strong>
                                    <small>Advanced technical content</small>
                                </span>
                            </span>
                        </label>
                    </div>
                )}

                {/* Button Group */}
                <div className={styles.buttonGroup}>
                    {step === 2 && (
                        <button className={styles.backButton} onClick={handleBack}>
                            Back
                        </button>
                    )}
                    <button className={styles.cancelButton} onClick={handleClose}>
                        Cancel
                    </button>
                    <button className={styles.nextButton} onClick={handleNext}>
                        {step === 1 ? 'Next' : '✨ Personalize'}
                    </button>
                </div>
            </div>
        </div>
    );
}
